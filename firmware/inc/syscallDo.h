#ifndef _SYSCALL_DO_H_
#define _SYSCALL_DO_H_


#ifdef __cplusplus
extern "C" {
#endif

#ifdef _OS_BUILD_
    #error "Syscalls should not be called from OS code"
#endif

#include <cpu/inc/syscallDo.h>
#include <sensors.h>
#include <syscall.h>
#include <stdarg.h>
#include <gpio.h>
#include <osApi.h>
#include <seos.h>
#include <util.h>


/* it is always safe to use this, but using syscallDo0P .. syscallDo4P macros may produce faster code for free */
static inline uintptr_t syscallDoGeneric(uint32_t syscallNo, ...)
{
    uintptr_t ret;
    va_list vl;

    va_start(vl, syscallNo);
    #ifdef SYSCALL_PARAMS_PASSED_AS_PTRS
        ret = cpuSyscallDo(syscallNo, &vl);
    #else
        ret = cpuSyscallDo(syscallNo, *(uint32_t*)&vl);
    #endif
    va_end(vl);

    return ret;
}

#ifdef cpuSyscallDo0P
    #define syscallDo0P(syscallNo) cpuSyscallDo0P(syscallNo)
#else
    #define syscallDo0P(syscallNo) syscallDoGeneric(syscallNo)
#endif

#ifdef cpuSyscallDo1P
    #define syscallDo1P(syscallNo,p1) cpuSyscallDo1P(syscallNo,p1)
#else
    #define syscallDo1P(syscallNo,p1) syscallDoGeneric(syscallNo,p1)
#endif

#ifdef cpuSyscallDo2P
    #define syscallDo2P(syscallNo,p1,p2) cpuSyscallDo2P(syscallNo,p1,p2)
#else
    #define syscallDo2P(syscallNo,p1,p2) syscallDoGeneric(syscallNo,p1,p2)
#endif

#ifdef cpuSyscallDo3P
    #define syscallDo3P(syscallNo,p1,p2,p3) cpuSyscallDo3P(syscallNo,p1,p2,p3)
#else
    #define syscallDo3P(syscallNo,p1,p2,p3) syscallDoGeneric(syscallNo,p1,p2,p3)
#endif

#ifdef cpuSyscallDo4P
    #define syscallDo4P(syscallNo,p1,p2,p3,p4) cpuSyscallDo4P(syscallNo,p1,p2,p3,p4)
#else
    #define syscallDo4P(syscallNo,p1,p2,p3,p4) syscallDoGeneric(syscallNo,p1,p2,p3,p4)
#endif

#ifdef cpuSyscallDo5P
    #define syscallDo5P(syscallNo,p1,p2,p3,p4,p5) cpuSyscallDo5P(syscallNo,p1,p2,p3,p4,p5)
#else
    #define syscallDo5P(syscallNo,p1,p2,p3,p4,p5) syscallDoGeneric(syscallNo,p1,p2,p3,p4,p5)
#endif



//system syscalls live here
static inline bool eOsEventSubscribe(uint32_t tid, uint32_t evtType)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_SUBCRIBE), tid, evtType);
}

static inline bool eOsEventUnsubscribe(uint32_t tid, uint32_t evtType)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_UNSUBCRIBE), tid, evtType);
}

static inline bool eOsEnqueueEvt(uint32_t evtType, void *evtData, uint32_t tidOfWhoWillFreeThisEvent) // tidOfWhoWillFreeThisEvent is likely your TID
{
    return syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_ENQUEUE), evtType, evtData, tidOfWhoWillFreeThisEvent);
}

static inline void eOsLogvInternal(enum LogLevel level, const char *str, uintptr_t args_list)
{
    (void)syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_LOGGING, SYSCALL_OS_MAIN_LOG_LOGV), level, str, args_list);
}

static inline void eOsLogv(enum LogLevel level, const char *str, va_list vl)
{
    eOsLogvInternal(level, str, VA_LIST_TO_INTEGER(vl));
}

static inline void eOsLog(enum LogLevel level, const char *str, ...)
{
    va_list vl;

    va_start(vl, str);
    eOsLogvInternal(level, str, VA_LIST_TO_INTEGER(vl));
    va_end(vl);
}

static inline const struct SensorInfo* eOsSensorSignalInternalEvt(uint32_t handle, uint32_t intEvtNum, uint32_t value1, uint64_t value2)
{
    uint32_t value2_lo = value2;
    uint32_t value2_hi = value2 >> 32;

    return (const struct SensorInfo*)syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_SIGNAL), handle, intEvtNum, value1, value2_lo, value2_hi);
}

static inline const uint32_t eOsSensorRegister(const struct SensorInfo *si, uint32_t tid)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_REG), si, tid);
}

static inline const bool eOsSensorUnregister(uint32_t handle)
{
    return syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_UNREG), handle);
}

static inline const struct SensorInfo* eOsSensorFind(uint32_t sensorType, uint32_t idx, uint32_t *handleP)
{
    return (const struct SensorInfo*)syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_FIND), sensorType, idx, handleP);
}

static inline bool eOsSensorRequest(uint32_t clientId, uint32_t sensorHandle, uint32_t rate, uint64_t latency)
{
    uint32_t latency_lo = latency;
    uint32_t latency_hi = latency >> 32;

    return syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_REQUEST), clientId, sensorHandle, rate, latency_lo, latency_hi);
}

static inline bool eOsSensorRequestRateChange(uint32_t clientId, uint32_t sensorHandle, uint32_t newRate, uint64_t newLatency)
{
    uint32_t newLatency_lo = newLatency;
    uint32_t newLatency_hi = newLatency >> 32;

    return syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_RATE_CHG), clientId, sensorHandle, newRate, newLatency_lo, newLatency_hi);
}

static inline bool eOsSensorRelease(uint32_t clientId, uint32_t sensorHandle)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_RELEASE), clientId, sensorHandle);
}

static inline bool eOsSensorTriggerOndemand(uint32_t clientId, uint32_t sensorHandle)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_TRIGGER), clientId, sensorHandle);
}

static inline uint32_t eOsSensorGetCurRate(uint32_t sensorHandle)
{
    return syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_GET_RATE), sensorHandle);
}

static inline struct Gpio* eOsGpioRequest(uint32_t gpioNum)
{
    return (struct Gpio*)syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_REQ), gpioNum);
}

static inline void eOsGpioRelease(struct Gpio* __restrict gpio)
{
    syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_REL), gpio);
}

static inline void eOsGpioConfigInput(const struct Gpio* __restrict gpio, int32_t gpioSpeed, enum GpioPullMode pull)
{
    syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_CFG_IN), gpio, gpioSpeed, pull);
}

static inline void eOsGpioConfigOutput(const struct Gpio* __restrict gpio, int32_t gpioSpeed, enum GpioPullMode pull, enum GpioOpenDrainMode odrMode, bool value)
{
    syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_CFG_OUT), gpio, gpioSpeed, pull, odrMode, value);
}

static inline void eOsGpioConfigAlt(const struct Gpio* __restrict gpio, int32_t gpioSpeed, enum GpioPullMode pull, enum GpioOpenDrainMode odrMode, uint32_t altFunc)
{
    syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_CFG_ALT), gpio, gpioSpeed, pull, odrMode, altFunc);
}

static inline bool eOsGpioGet(const struct Gpio* __restrict gpio)
{
    return !!syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_GET), gpio);
}

static inline void eOsGpioSet(const struct Gpio* __restrict gpio, bool value)
{
    syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_GPIO, SYSCALL_OS_DRV_GPIO_SET), gpio, value);
}

static inline uint64_t eOsTimGetTime(void)
{
    uint64_t timeNanos;
    syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_TIME, SYSCALL_OS_MAIN_TIME_GET_TIME), &timeNanos);
    return timeNanos;
}

static inline int oEsI2cMasterRequest(uint32_t busId, uint32_t speedInHz)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_MASTER, SYSCALL_OS_DRV_I2CM_REQ), busId, speedInHz);
}

static inline int oEsI2cMasterRelease(uint32_t busId)
{
    return syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_MASTER, SYSCALL_OS_DRV_I2CM_REL), busId);
}

static inline int oEsI2cMasterTxRx(uint32_t busId, uint32_t addr, const void *txBuf, size_t txSize, void *rxBuf, size_t rxSize, uint32_t cbkTid, void *cookie)
{
    return syscallDoGeneric(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_MASTER, SYSCALL_OS_DRV_I2CM_TXRX), busId, addr, txBuf, txSize, rxBuf, rxSize, cbkTid, cookie);
}

static inline int oEsI2cSlaveRequest(uint32_t busId, uint32_t addr)
{
    return syscallDo2P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_SLAVE, SYSCALL_OS_DRV_I2CS_REQ), busId, addr);
}

static inline int oEsI2cSlaveRelease(uint32_t busId)
{
    return syscallDo1P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_SLAVE, SYSCALL_OS_DRV_I2CS_REL), busId);
}

static inline void oEsI2cSlaveEnableRx(uint32_t busId, void *rxBuf, size_t rxSize, uint32_t cbkTid, void *cookie)
{
    syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_SLAVE, SYSCALL_OS_DRV_I2CS_RX_EN), busId, rxBuf, rxSize, cbkTid, cookie);
}

static inline int oEsI2cSlaveTxPreamble(uint32_t busId, uint8_t byte, uint32_t cbkTid, void *cookie)
{
    return syscallDo4P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_SLAVE, SYSCALL_OS_DRV_I2CS_TX_PRE), busId, byte, cbkTid, cookie);
}

static inline int oEsI2cSlaveTxPacket(uint32_t busId, const void *txBuf, size_t txSize, uint32_t cbkTid, void *cookie)
{
    return syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_DRIVERS, SYSCALL_OS_DRV_I2C_SLAVE, SYSCALL_OS_DRV_I2CS_TX_PKT), busId, txBuf, txSize, cbkTid, cookie);
}




#ifdef __cplusplus
}
#endif

#endif

