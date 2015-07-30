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
#include <stdint.h>
#include <seos.h>

/* plat life cycle */
void platInitialize(void);
void platUninitialize(void);
void platReset(void);

/* CPU sleep/wake */
void platSleep(void);
void platWake(void);

/* Interrupts */
uint64_t platEnableInterrupts(void);
uint64_t platDisableInterrupts(void);
void platRestoreInterrupts(uint64_t state);

/* Logging */
void *platLogAllocUserData();
void platLogFlush(void *userData);
bool platLogPutcharF(void *userData, char ch);

/* fast timer */
uint64_t platGetTicks(void);
void platSetAlarm(unsigned delayUs);
void platCancelAlarm(void);

#ifdef __cplusplus
}
#endif

#endif

