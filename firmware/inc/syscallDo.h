#ifndef _SYSCALL_DO_H_
#define _SYSCALL_DO_H_


#ifdef __cplusplus
extern "C" {
#endif

#ifdef _OS_BUILD_
    #error "Syscalls should not be called from OS code"
#endif

#include <cpu/inc/syscallDo.h>
#include <syscall.h>
#include <stdarg.h>
#include <seos.h>


static inline uintptr_t syscallDo(uint32_t syscallNo, ...)
{
    uintptr_t ret;
    va_list vl;

    va_start(vl, syscallNo);
    ret = cpuSyscallDo(syscallNo, &vl);
    va_end(vl);

    return ret;
}

//system syscalls live here
static inline bool eOsEventSubscribe(uint32_t tid, uint32_t evtType)
{
    return syscallDo(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_SUBCRIBE), tid, evtType);
}

static inline bool eOsEventUnsubscribe(uint32_t tid, uint32_t evtType)
{
    return syscallDo(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_UNSUBCRIBE), tid, evtType);
}

static inline bool eOsEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external)
{
    return syscallDo(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_ENQUEUE), evtType, evtData, evtFreeF, external);
}

static inline bool eOsDefer(OsDeferCbkF callback, void *cookie)
{
    return syscallDo(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_EVENTQ, SYSCALL_OS_MAIN_EVTQ_FUNC_DEFER), callback, cookie);
}

static inline void eOsLogv(enum LogLevel level, const char *str, va_list vl)
{
    (void)syscallDo(SYSCALL_NO(SYSCALL_DOMAIN_OS, SYSCALL_OS_MAIN, SYSCALL_OS_MAIN_LOGGING, SYSCALL_OS_MAIN_LOG_LOGV), level, str, &vl);
}

static inline void eOsLog(enum LogLevel level, const char *str, ...)
{
    va_list vl;

    va_start(vl, str);
    eOsLogv(level, str, vl);
    va_end(vl);
}


#ifdef __cplusplus
}
#endif

#endif

