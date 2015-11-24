#include <stdlib.h>
#include <string.h>
#include <float.h>

#include <eventnums.h>
#include <gpio.h>
#include <heap.h>
#include <hostIntf.h>
#include <isr.h>
#include <nanohubPacket.h>
#include <sensors.h>
#include <seos.h>
#include <timer.h>
#include <plat/inc/gpio.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>
#include <variant/inc/variant.h>

#ifndef VSYNC_PIN
#error "VSYNC_PIN is not defined; please define in variant.h"
#endif

#ifndef VSYNC_IRQ
#error "VSYNC_IRQ is not defined; please define in variant.h"
#endif

static struct SensorTask
{
    struct Gpio *pin;
    struct ChainedIsr isr;

    uint32_t id;
    uint32_t sensorHandle;

    bool on;
} mTask;

static bool vsyncIsr(struct ChainedIsr *localIsr)
{
    struct SensorTask *data = container_of(localIsr, struct SensorTask, isr);
    union EmbeddedDataPoint sample;

    if (!extiIsPendingGpio(data->pin)) {
        return false;
    }

    if (data->on) {
        sample.idata = 1;
        osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_VSYNC), sample.vptr, NULL);
    }

    extiClearPendingGpio(data->pin);
    return true;
}

static bool enableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    gpioConfigInput(pin, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(pin);
    extiEnableIntGpio(pin, EXTI_TRIGGER_FALLING);
    extiChainIsr(VSYNC_IRQ, isr);
    return true;
}

static bool disableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    extiUnchainIsr(VSYNC_IRQ, isr);
    extiDisableIntGpio(pin);
    return true;
}

static const struct SensorInfo mSensorInfo =
{
    "Camera Vsync",
    NULL,
    SENS_TYPE_VSYNC,
    NUM_AXIS_EMBEDDED,
    NANOHUB_INT_NONWAKEUP,
    20,
};

static bool vsyncPower(bool on)
{
    if (on) {
        extiClearPendingGpio(mTask.pin);
        enableInterrupt(mTask.pin, &mTask.isr);
    } else {
        disableInterrupt(mTask.pin, &mTask.isr);
        extiClearPendingGpio(mTask.pin);
    }

    mTask.on = on;
    sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);
    return true;
}

static bool vsyncFirmwareUpload()
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
}

static bool vsyncSetRate(uint32_t rate, uint64_t latency)
{
    return sensorSignalInternalEvt(mTask.sensorHandle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
}

static bool vsyncFlush()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_VSYNC), SENSOR_DATA_EVENT_FLUSH, NULL);
}

static const struct SensorOps mSensorOps =
{
    vsyncPower,
    vsyncFirmwareUpload,
    vsyncSetRate,
    vsyncFlush,
    NULL
};

static void handleEvent(uint32_t evtType, const void* evtData)
{
}

static bool startTask(uint32_t taskId)
{
    osLog(LOG_INFO, "VSYNC: task starting\n");

    mTask.id = taskId;
    mTask.sensorHandle = sensorRegister(&mSensorInfo, &mSensorOps);
    sensorRegisterInitComplete(mTask.sensorHandle);
    mTask.pin = gpioRequest(VSYNC_PIN);
    mTask.isr.func = vsyncIsr;

    return true;
}

static void endTask(void)
{
    disableInterrupt(mTask.pin, &mTask.isr);
    extiUnchainIsr(VSYNC_IRQ, &mTask.isr);
    extiClearPendingGpio(mTask.pin);
    gpioRelease(mTask.pin);
    sensorUnregister(mTask.sensorHandle);
    memset(&mTask, 0, sizeof(struct SensorTask));
}

INTERNAL_APP_INIT(0x0000000000000007ULL, startTask, endTask, handleEvent);
