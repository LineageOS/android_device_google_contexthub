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
    GPIO_MODE_ANALOG,
} GpioMode;

typedef enum
{
    GPIO_OUT_PUSH_PULL = 0,
    GPIO_OUT_OPEN_DRAIN,
} GpioOpenDrainMode;

typedef enum
{
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} GpioPullMode;

typedef uint8_t GpioNum;
struct Gpio;

/* Requests a GPIO and populates the gpio struct */
void gpioRequest(struct Gpio* __restrict gpio, GpioNum number);

/* Configures the direction and pull type of a GPIO */
void gpioConfig(const struct Gpio* __restrict gpio, GpioMode mode, GpioPullMode pull);
void gpioConfig_output(const struct Gpio* __restrict gpio, GpioOpenDrainMode mode);

/* Sets and gets a value for a specific GPIO pin */
void gpioSet(const struct Gpio* __restrict gpio, bool value);
bool gpioGet(const struct Gpio* __restrict gpio);

#ifdef __cplusplus
}
#endif

#endif

