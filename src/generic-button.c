#include "generic-button.h"

#include <string.h>

#include "freertos/semphr.h"
#include "esp_log.h"

static const char* TAG = "button";

static bool isr_service_installed = false;
static uint32_t button_count = 0;
static SemaphoreHandle_t button_mutex = NULL;

typedef struct button_t {
    gpio_num_t pin;
    button_idle_state_t idle_state;
    QueueHandle_t event_queue;
    uint32_t last_interrupt_time;
    TickType_t debounce_ticks;
} button_t;

static void IRAM_ATTR button_isr_handler(void* arg) {
    button_t* button = (button_t*) arg;

    uint32_t current_time = xTaskGetTickCountFromISR();
    if(current_time - button->last_interrupt_time < button->debounce_ticks) {
        return;
    }
    button->last_interrupt_time = current_time;

    button_event_type_t event = (gpio_get_level(button->pin) != button->idle_state) ? BUTTON_EVENT_PRESS : BUTTON_EVENT_RELEASE;
    xQueueSendFromISR(button->event_queue, &event, NULL);
}

static esp_err_t mutex_init() {
    if(!button_mutex) {
        button_mutex = xSemaphoreCreateMutex();
        if(!button_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

esp_err_t button_init(button_handle_t* button_handle, const button_config_t* button_config) {
    if(!button_handle || !button_config) {
        ESP_LOGE(TAG, "Invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    
    if(mutex_init() != ESP_OK) {
        return ESP_ERR_NO_MEM;
    }

    button_t* button = (button_t*) (malloc(sizeof(button_t)));
    if(!button) {
        ESP_LOGE(TAG, "Failed to allocate memory for button gpio %d", button_config->gpio_pin);
        return ESP_ERR_NO_MEM;
    }
    memset(button, 0, sizeof(button_t));

    button->pin = button_config->gpio_pin;
    button->debounce_ticks = (!button_config->debounce_ms) ? pdMS_TO_TICKS(BUTTON_DEBOUNCE_DEFAULT_MS) : pdMS_TO_TICKS(button_config->debounce_ms);

    gpio_config_t gpio_cfg = {
        .pin_bit_mask  = (1ULL << button->pin),
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
    esp_err_t ret = gpio_config(&gpio_cfg);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gpio %d", button->pin);
        goto ERROR;
    }

    if(button_config->idle_state == BUTTON_STATE_IDLE_DEFAULT) {
        uint32_t level = gpio_get_level(button->pin);
        button->idle_state = (level) ? BUTTON_STATE_IDLE_HIGH : BUTTON_STATE_IDLE_LOW;
    } else {
        button->idle_state = button_config->idle_state;
    }

    button->event_queue = (!button_config->queue_size) ? xQueueCreate(BUTTON_QUEUE_SIZE_DEFAULT, sizeof(button_event_type_t)) : xQueueCreate(button_config->queue_size, sizeof(button_event_type_t));
    if(!button->event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue for gpio %d", button->pin);
        goto ERROR;
    }

    bool mutex_taken = false;
    if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE) {
        mutex_taken = true;
        if(!isr_service_installed) {
            ret = gpio_install_isr_service(0);
            if(ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to install ISR service");
                goto ERROR;
            }
            isr_service_installed = true;
        }    
        ret = gpio_isr_handler_add(button->pin, button_isr_handler, button);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler for gpio %d", button->pin);
            goto ERROR;
        }

        button_count++;
        xSemaphoreGive(button_mutex);
        mutex_taken = false;
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        ret = ESP_ERR_TIMEOUT;
        goto ERROR;
    }

    *button_handle = button;
    ESP_LOGI(TAG, "Button initialized on GPIO %d", button->pin);
    return ESP_OK;

ERROR:
    if(button->event_queue) {
        vQueueDelete(button->event_queue);
    }
    if(mutex_taken) {
        xSemaphoreGive(button_mutex);
    }
    free(button);
    return ret;
}

esp_err_t button_free(button_handle_t* button_handle) {
    if(!button_handle || !*button_handle) {
        ESP_LOGE(TAG, "Invalid button handle");
        return ESP_ERR_INVALID_ARG;
    }
    button_t* button = (button_t*) *button_handle;

    esp_err_t ret = gpio_isr_handler_remove(button->pin);
    if(ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to remove ISR handler for gpio %d", button->pin);
    }

    if(button->event_queue) {
        vQueueDelete(button->event_queue);
    }

    if(xSemaphoreTake(button_mutex, portMAX_DELAY) == pdTRUE) {
        button_count--;
        
        if(!button_count && isr_service_installed) {
            gpio_uninstall_isr_service();
            isr_service_installed = false;
        }
        xSemaphoreGive(button_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take mutex during free");
    }

    ret = gpio_reset_pin(button->pin);
    if(ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reset gpio %d", button->pin);
    }

    if(!button_count && button_mutex) {
        vSemaphoreDelete(button_mutex);
        button_mutex = NULL;
    }

    free(button);
    *button_handle = NULL;
    return ESP_OK;
}

QueueHandle_t button_get_event_queue(button_handle_t button_handle) {
    if(!button_handle) {
        ESP_LOGE(TAG, "Invalid button handle");
        return NULL;
    }
    return ((button_t*) button_handle)->event_queue;
}

int button_get_level(button_handle_t button_handle) {
    if(!button_handle) {
        ESP_LOGE(TAG, "Invalid button handle");
        return -1;
    }
    button_t* button = (button_t*) button_handle;
    return gpio_get_level(button->pin);
}