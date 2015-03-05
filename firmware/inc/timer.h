#ifndef _TIMER_H_
#define _TIMER_H_

//
//  timer.h
//  seos
//
//  Created by Simon Wilson on 10/7/14.
//  Copyright (c) 2014 Google. All rights reserved.
//

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define NS_PER_S 1000000000ULL

struct timer_item_t
{
    //TODO: Pack these so sensor_type doesn't waste space
    //TODO: should these all be nanotimes?
    task_t *task;
    nanotime_t deadline;
    nanotime_t ideal_delay;
    nanosec_t max_jitter_ns;
    nanosec_t max_drift_ns;
    bool one_shot;
} typedef timer_item_t;

enum timer_error_t
{
    TIMER_ERR_TOO_BIG = 1,
    TIMER_ERR_TOO_SMALL,
    TIMER_ERR_ACCURACY_REQUIREMENTS_UNMET,
    TIMER_ERR_EVERYTHING_IS_TERRIBLE,
};

void Timer_init(void);
bool Timer_insert_timer(timer_item_t timer);
bool Timer_insert(task_t *task, nanotime_t delay, nanosec_t max_jitter_ns,
                  nanosec_t max_drift_ns, bool one_shot);
void Timer_interrupt_handler(void);
timer_item_t Timer_expire_next(void);
bool Timer_is_active(void);
timer_item_t *Timer_earliest(void);
void Timer_clear_timers_for_task(task_t *task);

/* Nanotime helper functions.  TODO: May want to move somewhere else. */
bool nanotime_less_than(nanotime_t time_a, nanotime_t time_b);
nanotime_t nanotime_add(nanotime_t time_a, nanotime_t time_b);
nanotime_t nanotime_subtract(nanotime_t time_a, nanotime_t time_b);
uint64_t nanotime_to_us(nanotime_t time);

#ifdef __cplusplus
}
#endif

#endif

