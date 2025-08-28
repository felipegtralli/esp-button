#pragma once

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

typedef struct button_t* button_handle_t;

typedef enum {
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE
} button_event_type_t;

typedef struct {
    gpio_num_t gpio_pin;
    button_event_type_t type;
    TickType_t timestamp;
} button_event_t;