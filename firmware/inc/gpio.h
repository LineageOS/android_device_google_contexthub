#ifndef GPIO_H_
#define GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

enum GpioMode
{
    GPIO_MODE_IN = 0,
    GPIO_MODE_OUT,
    GPIO_MODE_ALTERNATE,
    GPIO_MODE_ANALOG,
};

enum GpioOpenDrainMode
{
    GPIO_OUT_PUSH_PULL = 0,
    GPIO_OUT_OPEN_DRAIN,
};

enum GpioPullMode
{
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
};

enum GpioSpeed;
enum GpioAltFunc;
typedef uint8_t GpioNum;
struct Gpio;

/* Requests a GPIO and allocates the gpio handle/struct/etc */
struct Gpio* gpioRequest(GpioNum number);
void gpioRelease(struct Gpio* __restrict gpio);

/* Configures the direction and pull type of a GPIO */
void gpioConfigInput(const struct Gpio* __restrict gpio, enum GpioSpeed speed, enum GpioPullMode pull);
void gpioConfigOutput(const struct Gpio* __restrict gpio, enum GpioSpeed speed, enum GpioPullMode pull, enum GpioOpenDrainMode output, bool value);
void gpioConfigAlt(const struct Gpio* __restrict gpio, enum GpioSpeed speed, enum GpioPullMode pull, enum GpioOpenDrainMode output, enum GpioAltFunc func);

/* Sets and gets a value for a specific GPIO pin */
void gpioSet(const struct Gpio* __restrict gpio, bool value);
bool gpioGet(const struct Gpio* __restrict gpio);

#ifdef __cplusplus
}
#endif

#endif
