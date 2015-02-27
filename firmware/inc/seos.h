/*
 * SEOS: Sensor Event Operating System
 * An event-driven cooperative multitasking system for sensor hubs
 * Author: Simon Wilson
 */

#ifndef _SEOS_H_
#define _SEOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <list.h>


struct task_t;
struct event_t;

typedef unsigned sec_t;
typedef unsigned nanosec_t;
typedef int timer_handle_t;

enum interrupt {
    INTERRUPT_ALARM = 0,
    INTERRUPT_SENSOR,
    INTERRUPT_HALT,
    INTERRUPT_MAX,
};

#define NUM_EVENT_TYPES 9
/* These are numbered as such for use in a bit mask */
enum event_type_t {
    EVENT_NULL = 0,
    EVENT_TIMER_INTERRUPT,
    EVENT_TIMER,
    EVENT_HALT,
    EVENT_RESET,
    EVENT_SENSOR,
    EVENT_INTERRUPT,
} typedef event_type_t;

enum event_flags_t {
    EVENT_FLAG_NONE,
    EVENT_FLAG_PRODUCER_ENABLE,
    EVENT_FLAG_PRODUCER_DISABLE,
    EVENT_FLAG_PUBLISH,
} typedef event_flags_t;

/* Application lifecycle */
typedef void (*APP_start_task)(struct task_t *task);
typedef void (*APP_end_task)(struct task_t *task);

/* Application event loop */
typedef bool (*APP_handle_event)(struct task_t *task, event_type_t event_type);

struct nanotime_t {
    sec_t time_s;
    nanosec_t time_ns;
} typedef nanotime_t;

struct task_t {
    /* Do not include spaces in the name.*/
    char name[16];

    /* Subscribed events */
    int event_mask;

    /* App entry points */
    APP_start_task _APP_start_task;
    APP_end_task _APP_end_task;
    APP_handle_event _APP_handle_event;
} typedef task_t;

struct event_data_t {
    nanotime_t timestamp;
    void *data;
    int datalen;
} typedef event_data_t;

struct task_wakeup_t {
    event_type_t event_type;
    event_flags_t flags;
    task_t *task;
    nanotime_t deadline;
} typedef task_wakeup_t;

struct event_subscription_t {
    nanosec_t latency_ns;
    nanosec_t jitter_ns;
    nanosec_t drift_ns;
    nanotime_t period;
} typedef event_subscription_t;

/* Kernel */
void OS_initialize(void);
void OS_uninitialize(void);
void OS_scheduler(void);
void OS_halt(void);

/* Event queue */
//TODO: Data should be accessed in batches, this API may not capture that
void OS_event_data_enqueue(event_data_t event_data, event_type_t event_type);
event_data_t OS_event_data_dequeue(event_type_t event_type);
task_t *OS_get_task(char *taskname);

/* Task queue */
void OS_task_enqueue(task_wakeup_t task_wakeup);
bool OS_task_queue_empty(void);
task_wakeup_t OS_task_dequeue(void);

/* Interrupts */
typedef void (*interrupt_handler_t)(void *, unsigned);
void OS_timer_interrupt_handler(void);

/* Timer */
bool OS_add_timer_one_shot(task_t *task, nanotime_t delay,
                           nanosec_t max_drift_ns,
                           nanosec_t window_ns);
bool OS_add_timer_periodic(task_t *task,
                           nanotime_t period,
                           nanosec_t max_jitter_ns,
                           nanosec_t max_drift_ns);
void OS_delete_timer(struct task_t *task, timer_handle_t timer_handle);

/* Clocks */
unsigned OS_get_systick(void);
/* Returns the number of seconds and nanoseconds that have transpired
 * since Epoch (January 1, 1970). */
nanotime_t OS_get_time(void);

/* Events */
bool OS_event_subscribe(task_t *task, event_type_t event_type,
                        nanosec_t latency_ns, nanosec_t jitter_ns,
                        nanosec_t drift_ns, nanotime_t period);
void OS_unsubscribe_to_task(task_t *task, event_type_t);


/* TODO: May want to switch to macros so they can be compiled out. */
/* Logging */
enum log_level_t {
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG,
};

void OS_log(enum log_level_t level, char *str);

void OS_main(void);
void OS_interrupt_handler(enum interrupt interrupt, interrupt_handler_t handler);
void OS_interrupt(enum interrupt interrupt, void *data, unsigned len);
void OS_cancel_task_timers(struct task_t *task);

//TODO: better place for this
void APP_register_task0(struct task_t *task);
void APP_register_task1(struct task_t *task);

#ifdef __cplusplus
}
#endif

#endif

