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

static inline bool eOsEnqueueEvt(uint32_t evtType, void *evtData, uint32_t tidOfWhoWillFreeThisEvent, bool external) // tidOfWhoWillFreeThisEvent is likely your TID
{
    return syscallDo4P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_ENQUEUE), evtType, evtData, tidOfWhoWillFreeThisEvent, external);
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
    uint64_t value2_hi = value2 >> 32;

    return (const struct SensorInfo*)syscallDo5P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_SIGNAL), handle, intEvtNum, value1, value2_lo, value2_hi);
}

static inline const struct SensorInfo* eOsSensorFind(uint32_t sensorType, uint32_t idx, uint32_t *handleP)
{
    return (const struct SensorInfo*)syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_FIND), sensorType, idx, handleP);
}

static inline bool eOsSensorRequest(uint32_t clientId, uint32_t sensorHandle, uint32_t rate)
{
    return syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_REQUEST), clientId, sensorHandle, rate);
}

static inline bool eOsSensorRequestRateChange(uint32_t clientId, uint32_t sensorHandle, uint32_t newRate)
{
    return syscallDo3P(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_SENSOR, SYSCALL_OS_MAIN_SENSOR_RATE_CHG), clientId, sensorHandle, newRate);
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


#ifdef __cplusplus
}
#endif

#endif

