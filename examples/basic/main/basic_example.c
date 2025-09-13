#include "esp_button.h"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define BUTTON_GPIO_NUM CONFIG_BUTTON_GPIO_NUM

static const char TAG[] = "basic_example";

// User-defined callback function to handle button events
static void button_callback(button_handle_t button, const button_event_t* event, void* ctx) {
    switch(event->type) {
        case BUTTON_EVENT_PRESS:
            ESP_LOGI(TAG, "Button pressed GPIO %d. Timestamp: %lu", event->gpio_pin, pdTICKS_TO_MS(event->timestamp));
            break;
        case BUTTON_EVENT_RELEASE:
            ESP_LOGI(TAG, "Button released GPIO %d. Timestamp: %lu", event->gpio_pin, pdTICKS_TO_MS(event->timestamp));
            break;
        default:
            ESP_LOGW(TAG, "Unknown event");
            break;
    }
}

void app_main() {
    const button_config_t button_config = {
        .gpio_pin = BUTTON_GPIO_NUM,
        .pull_mode = GPIO_PULLUP_ONLY,
        .idle_state = BUTTON_STATE_IDLE_DEFAULT, // Auto-detect idle state. Use BUTTON_STATE_IDLE_LOW or BUTTON_STATE_IDLE_HIGH if you know the idle state.
        .debounce_ms = BUTTON_DEBOUNCE_DEFAULT_MS, // Use default debounce time (20ms). If set to 0 then BUTTON_DEBOUNCE_DEFAULT_MS is used.
        .callback = button_callback, // User-defined callback function
        .user_ctx = NULL,
    };

    button_handle_t button = NULL;
    ESP_ERROR_CHECK(button_init(&button, &button_config)); // Use button_free() after use
}