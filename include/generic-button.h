#pragma once

#include "generic-button-events.h"

#include "freertos/FreeRTOS.h"

#include "esp_err.h"
#include "driver/gpio.h"

#define BUTTON_DEBOUNCE_DEFAULT_MS 20

/**
 * @brief Button idle state type
 * 
 */
typedef enum {
    BUTTON_STATE_IDLE_LOW,
    BUTTON_STATE_IDLE_HIGH,
    BUTTON_STATE_IDLE_DEFAULT
} button_idle_state_t;

/**
 * @brief Opaque button handle type
 * 
 */
typedef struct button_t* button_handle_t;

/**
 * @brief Button callback function type
 * 
 * @param button Button handle that triggered the event
 * @param event Pointer to button event (PRESS/RELEASE)
 * @param ctx User context pointer provided during button initialization
 * 
 * @note Callback is invoked from the button event task context, not ISR.
 * 
 */
typedef void (*button_callback_t)(button_handle_t button, button_event_t* event, void* ctx);

/**
 * @brief Button configuration structure
 * 
 * @note Debounce is applied in the event task, not ISR.
 *       Debounce = 0 will use the default debounce time (BUTTON_DEBOUNCE_DEFAULT_MS).
 * 
 */
typedef struct {
    gpio_num_t gpio_pin;
    gpio_pull_mode_t pull_mode;
    button_idle_state_t idle_state;
    uint32_t debounce_ms; //0 = BUTTON_DEBOUNCE_DEFAULT_MS
    button_callback_t callback;
    void* user_ctx;
} button_config_t;

/**
 * @brief Initialize button
 * 
 * @param[out] button_handle Pointer to button handle to be initialized
 * @param[in] button_config Pointer to button configuration structure
 * 
 * @return 
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if invalid arguments
 *      - ESP_ERR_NO_MEM if memory allocation fails
 *      - Other error codes from underlying functions
 */
esp_err_t button_init(button_handle_t* button_handle, const button_config_t* button_config);

/**
 * @brief Free button resources
 * 
 * @param[in,out] button_handle Pointer to button handle to be freed.
 *                              Handle is set to NULL on success.
 * 
 * @return 
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if invalid button handle
 *      - ESP_ERR_TIMEOUT if button event task did not terminate in time (resources leaked)
 *      - Other error codes from underlying functions
 */
esp_err_t button_free(button_handle_t* button_handle);

/**
 * @brief Get current button level
 * 
 * @param[in] button_handle Button handle
 * 
 * @return 
 *      - Current GPIO level (0 or 1) on success
 *      - -1 if invalid button handle
 * 
 * @note This reads the raw GPIO level directly, not the debounced state.
 * 
 */
int button_get_level(button_handle_t button_handle);