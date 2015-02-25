#ifndef _PLATFORM_H_
#define _PLATFORM_H_

//
//  platform.h
//  seos
//
//  Created by Simon Wilson on 10/2/14.
//  Copyright (c) 2014 Google. All rights reserved.
//

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <seos.h>

/* Test function typedef */
typedef bool (*TestFunc) (void);

/* Platform life cycle */
void Platform_initialize(void);
void Platform_uninitialize(void);
void Platform_reset(void);

/* CPU sleep/wake */
void Platform_sleep(void);
void Platform_wake(void);

/* Interrupts */
void Platform_enable_interrupts(void);
void Platform_disable_interrupts(void);

/* Logging */
void Platform_log(char *string);

/* RTC/alarm */
unsigned Platform_get_rtc_ms(void);
void Platform_set_alarm(unsigned delay_us);
void Platform_cancel_alarm(void);
unsigned Platform_get_systick(void);

#ifdef __cplusplus
}
#endif

#endif

