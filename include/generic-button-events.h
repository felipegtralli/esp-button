#pragma once

#include "generic-button.h"

#include "driver/gpio.h"

typedef enum {
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE
} button_event_type_t;

typedef struct {
    gpio_num_t gpio_pin;
    button_event_type_t type;
} button_event_t;