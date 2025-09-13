#include "esp_button.h"

#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#if CONFIG_LOG_DEFAULT_LEVEL >= 4 // ESP_LOG_DEBUG
    #define BUTTON_DEBUG
#endif

#define BUTTON_TASK_STACK_SIZE CONFIG_BUTTON_TASK_STACK_SIZE
#define BUTTON_TASK_PRIORITY CONFIG_BUTTON_TASK_PRIORITY
#define BUTTON_QUEUE_SIZE CONFIG_BUTTON_QUEUE_SIZE

#define BUTTON_INIT_FLAG_ISR_HANDLER_ADDED (1 << 0)
#define BUTTON_INIT_FLAG_MUTEX_TAKEN (1 << 1)
#define BUTTON_INIT_FLAG_COUNT_INCREMENTED (1 << 2)

#define MAX_LOOP 100

static const char TAG[] = "button";

static bool isr_service_installed = false;
static uint32_t button_count = 0;
static TaskHandle_t button_event_task_handle = NULL;
static QueueHandle_t button_event_queue = NULL;
static SemaphoreHandle_t button_mutex = NULL;

typedef struct button_t {
    gpio_num_t pin;
    button_idle_state_t idle_state;
    uint32_t last_interrupt_time;
    TickType_t debounce_ticks;
    button_callback_t callback;
    void* user_ctx;
} button_t;

typedef struct {
    button_t* button; // if NULL = shutdown signal
    button_event_t event;
} button_global_event_t;

static void IRAM_ATTR button_isr_handler(void* arg) {
    button_t* button = (button_t*)arg;
    if(!button || !button_event_queue) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    button_event_t event = {
        .gpio_pin = button->pin,
        .type = (gpio_get_level(button->pin) != button->idle_state) ? BUTTON_EVENT_PRESS : BUTTON_EVENT_RELEASE,
        .timestamp = xTaskGetTickCountFromISR()
    };
    button_global_event_t button_event = {
        .button = button,
        .event = event
    };
    (void)xQueueSendFromISR(button_event_queue, &button_event, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void button_event_task(void* arg) {
    button_global_event_t event;

    while(true) {
        if(xQueueReceive(button_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if(!event.button) {
                #ifdef BUTTON_DEBUG
                    ESP_LOGD(TAG, "Button event task received shutdown signal");
                #endif
                break; // shutdown signal
            }

            button_t* button = event.button;
            if(event.event.timestamp - button->last_interrupt_time > button->debounce_ticks) {
                if(button->callback) {
                    button->callback(button, &event.event, button->user_ctx);
                }
                button->last_interrupt_time = event.event.timestamp;
            }
        }
    }
    button_event_task_handle = NULL;
    vTaskDelete(NULL);
}

// init helpers
static esp_err_t button_config_gpio(const gpio_num_t gpio, const button_config_t* button_config) {
    gpio_config_t gpio_cfg = {
        .pin_bit_mask  = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    switch(button_config->pull_mode) {
        case GPIO_PULLUP_ONLY:
            gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case GPIO_PULLDOWN_ONLY:
            gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        case GPIO_PULLUP_PULLDOWN:
            gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        case GPIO_FLOATING:
            gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        default:
            gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
    }

    return gpio_config(&gpio_cfg);
}

static esp_err_t button_apply_config(button_t* button, const button_config_t* button_config) {
    button->pin = button_config->gpio_pin;
    button->debounce_ticks = (!button_config->debounce_ms) ? pdMS_TO_TICKS(BUTTON_DEBOUNCE_DEFAULT_MS) : pdMS_TO_TICKS(button_config->debounce_ms);

    if(button_config->idle_state == BUTTON_STATE_IDLE_DEFAULT) {
        int level = gpio_get_level(button->pin);
        button->idle_state = (level) ? BUTTON_STATE_IDLE_HIGH : BUTTON_STATE_IDLE_LOW;
        
        #ifdef BUTTON_DEBUG
            ESP_LOGD(TAG, "Detected idle state for GPIO %d as %s", button->pin, (button->idle_state == BUTTON_STATE_IDLE_HIGH) ? "HIGH" : "LOW");
        #endif
    } else {
        button->idle_state = button_config->idle_state;
    }

    if(button_config->callback) {
        button->callback = button_config->callback;
        button->user_ctx = button_config->user_ctx;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t button_queue_init(void) {
    if(!button_event_queue) {
        button_event_queue = xQueueCreate(BUTTON_QUEUE_SIZE, sizeof(button_global_event_t));
        if(!button_event_queue) {
            return ESP_ERR_NO_MEM;
        }

        #ifdef BUTTON_DEBUG
            ESP_LOGD(TAG, "Button event queue created. Size %d", BUTTON_QUEUE_SIZE);
        #endif
    }
    return ESP_OK;
}

static esp_err_t button_mutex_init(void) {
    if(!button_mutex) {
        button_mutex = xSemaphoreCreateMutex();
        if(!button_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static esp_err_t button_setup_isr_service(button_t* button, uint8_t* flags) {
    esp_err_t ret = ESP_OK;

    if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE) {
        *flags |= BUTTON_INIT_FLAG_MUTEX_TAKEN;
        if(!isr_service_installed) {
            ret = gpio_install_isr_service(0);
            if(ret != ESP_OK) {
                return ret;
            }
            isr_service_installed = true;
        }

        ret = gpio_isr_handler_add(button->pin, button_isr_handler, button);
        if(ret != ESP_OK) {
            return ret;
        }
        *flags |= BUTTON_INIT_FLAG_ISR_HANDLER_ADDED;

        button_count++;
        *flags |= BUTTON_INIT_FLAG_COUNT_INCREMENTED;
        xSemaphoreGive(button_mutex);
        *flags &= ~BUTTON_INIT_FLAG_MUTEX_TAKEN;
    } else {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t button_task_init(void) {
    if(!button_event_task_handle) {
        if(xTaskCreate(button_event_task, "button_event_task", BUTTON_TASK_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, &button_event_task_handle) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }

        #ifdef BUTTON_DEBUG
            ESP_LOGD(TAG, "Button event task created. Stack size %d, Priority %d", BUTTON_TASK_STACK_SIZE, BUTTON_TASK_PRIORITY);
        #endif
    }
    return ESP_OK;
}

static void button_cleanup_on_error(button_t* button, uint8_t flags) {
    if(flags & BUTTON_INIT_FLAG_ISR_HANDLER_ADDED) {
        (void)gpio_isr_handler_remove(button->pin);
    }
    if(flags & BUTTON_INIT_FLAG_COUNT_INCREMENTED) {
        if(flags & BUTTON_INIT_FLAG_MUTEX_TAKEN) {
            button_count--;
        } else if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE) {
            button_count--;
            (void)xSemaphoreGive(button_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to take mutex during cleanup");
        }
    }
    if(flags & BUTTON_INIT_FLAG_MUTEX_TAKEN) {
        (void)xSemaphoreGive(button_mutex);
    }
    if(button) {
        free(button);
    }
}

// free helpers
static esp_err_t button_global_cleanup(bool* mutex_delete) {
    if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE) {
        button_count--;

        if(button_count == 0) {
            if(isr_service_installed) {
                gpio_uninstall_isr_service();
                isr_service_installed = false;
            }
            if(button_event_task_handle) {
                button_global_event_t shutdown_event = { .button = NULL };
                (void)xQueueSend(button_event_queue, &shutdown_event, portMAX_DELAY);
            }
            *mutex_delete = true;
        }
        (void)xSemaphoreGive(button_mutex);
    } else {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t button_free_global_resources(void) {
    uint8_t loop_count = 0;
    while(button_event_task_handle && loop_count < MAX_LOOP) {
        vTaskDelay(pdMS_TO_TICKS(10));
        loop_count++;
    }

    if(button_event_task_handle) {
        return ESP_ERR_TIMEOUT;
    }

    if(button_event_queue) {
        vQueueDelete(button_event_queue);
        button_event_queue = NULL;
    }

    if(button_mutex) {
        vSemaphoreDelete(button_mutex);
        button_mutex = NULL;
    }

    return ESP_OK;
}

esp_err_t button_init(button_handle_t* button_handle, const button_config_t* button_config) {
    esp_err_t status = ESP_OK;
    if(!button_handle || !button_config) {
        ESP_LOGE(TAG, "Invalid argument");
        status = ESP_ERR_INVALID_ARG;
    }

    button_t* button = NULL;
    if(status == ESP_OK) {
        button = (button_t*)(malloc(sizeof(button_t)));
        if(!button) {
            ESP_LOGE(TAG, "Failed to allocate memory for button gpio %d", button_config->gpio_pin);
            status = ESP_ERR_NO_MEM;
        } else {
            memset(button, 0, sizeof(button_t));
        }
    }

    if(status == ESP_OK) {
        const gpio_num_t gpio = button_config->gpio_pin;
        status = button_config_gpio(gpio, button_config);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure gpio %d", gpio);
        }
    }

    if(status == ESP_OK) {
        status = button_apply_config(button, button_config);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Callback function is required");
        }
    }

    if(status == ESP_OK) {
        status = button_queue_init();
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create button event queue");
        }
    }

    if(status == ESP_OK) {
        status = button_mutex_init();
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create mutex");
        }
    }

    uint8_t init_flags = 0;
    if(status == ESP_OK) {
        status = button_setup_isr_service(button, &init_flags);
        if(status != ESP_OK) {
            if(status == ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "Failed to take mutex");
            } else {
                ESP_LOGE(TAG, "Failed to setup ISR for gpio %d. Error: %s", button->pin, esp_err_to_name(status));
            }
        }
    }

    if(status == ESP_OK) {
        status = button_task_init();
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create button event task");
        }
    }

    if(status == ESP_OK) {
        #ifdef BUTTON_DEBUG
            ESP_LOGD(TAG, "Button count incremented to %" PRIu32, button_count);
        #endif

        *button_handle = button;
        ESP_LOGI(TAG, "Button initialized on GPIO %d", button->pin);
    } else {
        button_cleanup_on_error(button, init_flags);
    }

    return status;
}

esp_err_t button_free(button_handle_t* button_handle) {
    esp_err_t status = ESP_OK;
    esp_err_t last_err = ESP_OK;

    button_t* button = NULL;
    if(!button_handle || !*button_handle) {
        ESP_LOGE(TAG, "Invalid button handle");
        status = ESP_ERR_INVALID_ARG;
        last_err = status;
    } else {
        button = (button_t*)*button_handle;
    }

    if(status == ESP_OK) {
        status = gpio_isr_handler_remove(button->pin);
        if(status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to remove ISR handler for gpio %d. ERROR: %s", button->pin, esp_err_to_name(status));
            last_err = status;
            status = ESP_OK;
        }
    }

    bool mutex_delete = false;
    if(status == ESP_OK) {
        status = button_global_cleanup(&mutex_delete);
        if(status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to take mutex during free");
            last_err = status;
            status = ESP_OK;
        }
    }

    if(status == ESP_OK) {
        status = gpio_reset_pin(button->pin);
        if(status != ESP_OK) {
            ESP_LOGW(TAG, "Failed to reset gpio %d. ERROR: %s", button->pin, esp_err_to_name(status));
            last_err = status;
            status = ESP_OK;
        }
    }

    if(status == ESP_OK) {
        free(button);
        *button_handle = NULL;

        #ifdef BUTTON_DEBUG
            ESP_LOGD(TAG, "Button count decremented to %" PRIu32, button_count);
        #endif
    }

    if(mutex_delete) {
        last_err = button_free_global_resources();
        if(last_err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Button event task did not terminate in time - leaking resources (queue and mutex)");
        }
    }

    status = last_err;
    return status;
}

int button_get_level(button_handle_t button_handle) {   
    int ret = 0;
    if(!button_handle) {
        ESP_LOGE(TAG, "Invalid button handle");
        ret = -1;
    }

    if(ret != -1) {
        button_t* button = (button_t*)button_handle;
        ret = gpio_get_level(button->pin);
    }

    return ret;
}