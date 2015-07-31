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

/* Logging */
void *platLogAllocUserData();
void platLogFlush(void *userData);
bool platLogPutcharF(void *userData, char ch);

/* fast timer */
uint64_t platGetTicks(void); //in nanoseconds since an arbitrary starting point in the past


/* sleep/wake */
#define PLAT_MAX_SLEEP_DEVS    32
enum PlatSleepDevID;

void platSleep(void);


//in platSleepClockRequest() code to set next timer of some variety will live
//note that maxErrTotalPpm != maxDriftPpm + maxJitterPpm is quite possible since it is possible to have:
// timer A allowing 300ppm of jitter and 10pp of drift and timer B allowing 20ppm of jitter and 500ppm of drift
// in that case we'd see maxJitterPpm = 200, maxDriftPpm = 500, maxErrTotalPpm = 520  (MAX of all timers' allowable error totals)
//return true if timer was set. false if you failed (you will be called right back though. so false is usually reserved for cases
// like "it is too soon to set a timer")
//a special case is when nextTimer == 0. this indicates no next timer, so configure system for that
//do not call this func if timer is already due - it will be delayed (potentially by a whole sleep-wke cycle), though this is unlikely
bool platSleepClockRequest(uint64_t wakeupTime, uint32_t maxJitterPpm, uint32_t maxDriftPpm, uint32_t maxErrTotalPpm); //"nextTime == 0" => "no wakeup needed"

/* 0 for any "max" value means "do not care" */
bool platRequestDevInSleepMode(uint32_t sleepDevID, uint32_t maxWakeupTime); //request that this device remain powered/clocked in sleep mode   (device lists are platform specific)
bool platReleaseDevInSleepMode(uint32_t sleepDevID); //unrequest that this device remain powered/clocked in sleep mode (device lists are platform specific)


#ifdef __cplusplus
}
#endif

#endif

