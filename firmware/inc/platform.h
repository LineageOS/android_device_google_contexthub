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
void platEnableInterrupts(void);
void platDisableInterrupts(void);

/* Output */
void platLogPutchar(char ch);

/* RTC */
uint64_t platSetRtcMs(void);
uint64_t platGetRtcMs(void);

/* fast timer */
uint64_t platGetTicks(void);
void platSetAlarm(unsigned delayUs);
void platCancelAlarm(void);

#ifdef __cplusplus
}
#endif

#endif

