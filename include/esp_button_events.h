#ifndef ESP_BUTTON_EVENTS_H
#define ESP_BUTTON_EVENTS_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

/**
 * @brief Button event types
 * 
 * Events are generated on state changes after debounce filtering.
 *  - BUTTON_EVENT_PRESS: Button pressed (state changed from idle to active)
 *  - BUTTON_EVENT_RELEASE: Button released (state changed from active to idle)
 * 
 */
typedef enum {
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE
} button_event_type_t;

/**
 * @brief Button event structure
 * 
 * @note Timestamp represents the moment the interrupt was triggered, measured in 
 *       FreeRTOS ticks. Convert to ms using pdTICKS_TO_MS().
 * 
 */
typedef struct {
    gpio_num_t gpio_pin;
    button_event_type_t type;
    TickType_t timestamp;
} button_event_t;

#endif // ESP_BUTTON_EVENTS_H