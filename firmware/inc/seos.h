/*
 * SEOS: Sensor Event Operating System
 * An event-driven cooperative multitasking system for sensor hubs
 * Author: Simon Wilson
 */

#ifndef _SEosH
#define _SEosH

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

struct entry_t {
    APP_start_task start_task;
    APP_end_task end_task;
    APP_handle_event handle_event;
} typedef entry_t;

struct task_t {
    /* Do not include spaces in the name.*/
    char name[16];

    /* Subscribed events */
    int event_mask;

    /* App entry points */
    struct entry_t funcs;
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
void osInitialize(void);
void osUninitialize(void);
void osScheduler(void);
void osHalt(void);

/* Event queue */
//TODO: Data should be accessed in batches, this API may not capture that
void osEventDataEnqueue(event_data_t eventData, event_type_t eventType);
event_data_t osEventDataDequeue(event_type_t eventType);
task_t *osGetTask(char *taskname);

/* Task queue */
void osTaskEnqueue(task_wakeup_t taskWakeup);
bool osTaskQueueEmpty(void);
task_wakeup_t osTaskDequeue(void);

/* Interrupts */
typedef void (*interrupt_handler_t)(void *, unsigned);
void osTimerInterruptHandler(void);

/* Timer */
bool osAddTimerOneShot(task_t *task, nanotime_t delay,
                           nanosec_t max_drift_ns,
                           nanosec_t window_ns);
bool osAddTimerPeriodic(task_t *task,
                           nanotime_t period,
                           nanosec_t max_jitter_ns,
                           nanosec_t max_drift_ns);
void osDeleteTimer(struct task_t *task, timer_handle_t timerHandle);

/* Clocks */
/* Returns the number of seconds and nanoseconds that have transpired
 * since Epoch (January 1, 1970). */
nanotime_t osGetTime(void);

/* Events */
bool osEventSubscribe(task_t *task, event_type_t eventType,
                        nanosec_t latency_ns, nanosec_t jitter_ns,
                        nanosec_t drift_ns, nanotime_t period);
void osUnsubscribeToTask(task_t *task, event_type_t);


/* TODO: May want to switch to macros so they can be compiled out. */
/* Logging */
enum log_level_t {
    LOG_ERROR = 'E',
    LOG_WARN = 'W',
    LOG_INFO = 'I',
    LOG_DEBUG = 'D',
};

void osLog(enum log_level_t level, const char *str, ...);

void osMain(void);
void osInterruptHandler(enum interrupt interrupt, interrupt_handler_t handler);
void osInterrupt(enum interrupt interrupt, void *data, unsigned len);
void osCancelTaskTimers(struct task_t *task);

#define APP_INIT(start, end, event) \
static entry_t __attribute__((used,section (".app_init"))) app_entry = {\
    .start_task = (start),\
    .end_task = (end),\
    .handle_event = (event)\
}

#ifdef __cplusplus
}
#endif

#endif
