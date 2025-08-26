#pragma once

#include "generic-button-events.h"

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "driver/gpio.h"

#define BUTTON_QUEUE_SIZE_DEFAULT 10
#define BUTTON_DEBOUNCE_DEFAULT_MS 20

typedef enum {
    BUTTON_STATE_IDLE_LOW,
    BUTTON_STATE_IDLE_HIGH,
    BUTTON_STATE_IDLE_DEFAULT
} button_idle_state_t;

typedef struct button_t* button_handle_t;

typedef struct {
    gpio_num_t gpio_pin;
    gpio_pull_mode_t pull_mode;
    button_idle_state_t idle_state;
    UBaseType_t queue_size;
    uint32_t debounce_ms;      
} button_config_t;

esp_err_t button_init(button_handle_t* button_handle, const button_config_t* button_config);
esp_err_t button_free(button_handle_t* button_handle);
QueueHandle_t button_get_event_queue(button_handle_t button_handle);
int button_get_level(button_handle_t button_handle);