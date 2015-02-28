#ifndef GPIO_H_
#define GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    GPIO_MODE_IN = 0,
    GPIO_MODE_OUT,
    GPIO_MODE_ALTERNATE,
    GPIO_MODE_ANALOG_,
} gpio_mode_t;

typedef enum
{
    GPIO_OUT_PUSH_PULL = 0,
    GPIO_OUT_OPEN_DRAIN,
} gpio_out_mode_t;

typedef enum
{
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} gpio_pull_t;

typedef uint8_t gpio_number_t;
struct gpio;

/* Requests a GPIO and populates the gpio struct */
void gpio_request(struct gpio* __restrict gpio, gpio_number_t number);

/* Configures the direction and pull type of a GPIO */
void gpio_configure(const struct gpio* __restrict gpio, gpio_mode_t mode, gpio_pull_t pull);
void gpio_configure_output(const struct gpio* __restrict gpio, gpio_out_mode_t mode);

/* Sets and gets a value for a specific GPIO pin */
void gpio_set_value(const struct gpio* __restrict gpio, bool value);
bool gpio_get_value(const struct gpio* __restrict gpio);

#ifdef __cplusplus
}
#endif

#endif

