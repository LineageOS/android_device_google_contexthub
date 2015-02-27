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

/* plat life cycle */
void platInitialize(void);
void platUninitialize(void);
void platReset(void);

/* CPU sleep/wake */
void platSleep(void);
void platWake(void);

/* Interrupts */
void platEnableInterrupts(void);
void platDisableInterrupts(void);

/* Logging */
void platLog(char *string);

/* RTC/alarm */
unsigned platGetRtcMs(void);
void platSetAlarm(unsigned delayUs);
void platCancelAlarm(void);
unsigned platGetSystick(void);

#ifdef __cplusplus
}
#endif

#endif

