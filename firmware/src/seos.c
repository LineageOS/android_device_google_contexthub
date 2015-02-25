#include <seos.h>
#include <platform.h>
#include <timer.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <heap.h>

void OS_idle_start_task(struct task_t *task);
void OS_idle_end_task(struct task_t *task);
bool OS_idle_handle_event(struct task_t *task, event_type_t event_type);


/*
 * TODO Hack: function to set up function pointers for statically
 * linked app. We need an app loader.
 */
extern void APP_register_task0(struct task_t *task);
extern void APP_register_task1(struct task_t *task);

//TODO:  only utilize this until real clock counter implemented.
extern nanotime_t timer_time;

extern TestFunc _stests[], _etests[];

/* Scheduler is running */
bool running;

/* Task list */
#define TASK_LIST_SIZE 16
struct task_t *task_list[TASK_LIST_SIZE];
unsigned num_tasks = 0;
unsigned cur_task = 0;

/* Idle task */
struct task_t idle_task;

#define MAX_SUBSCRIPTIONS 16
struct subscription_bucket_t {
    int items;
    task_t *producer;
    event_subscription_t subscriptions[MAX_SUBSCRIPTIONS];
    //TODO: Put data FIFO and tail maintenance here too
} typedef subscription_bucket_t;
subscription_bucket_t subscription_list[NUM_EVENT_TYPES] = {{0}};

#define TASK_WAKEUP_QUEUE_SIZE 16
task_wakeup_t task_wakeup_queue[TASK_WAKEUP_QUEUE_SIZE];
int task_wakeup_queue_head, task_wakeup_queue_tail = 0;


/* Interrupt handlers */
interrupt_handler_t interrupt_handler[INTERRUPT_MAX];


/*
 * Main entry point for OS
 */
void __attribute__((noreturn)) OS_main(void)
{
    heapInit();

    OS_initialize();

    OS_scheduler();

    OS_uninitialize();

    /* Exiting program undefined in embedded */
    while(1);
}

/*
 * Kernel
 */
void OS_initialize(void)
{
    Platform_disable_interrupts();

    Timer_init();
    Platform_initialize();

    OS_log(LOG_INFO, "SEOS Initializing\n");

    int i = 0;
    for (i = 0; i < TASK_LIST_SIZE; i++) {
        task_list[i] = NULL;
    }

    /* Set up idle task */
    strcpy(idle_task.name, "idle");
    idle_task.event_mask |= EVENT_NULL;
    idle_task._APP_start_task = OS_idle_start_task;
    idle_task._APP_end_task = OS_idle_end_task;
    idle_task._APP_handle_event = OS_idle_handle_event;

    /* Set up function pointers for statically linked apps. */
    task_list[0] = heapAlloc(sizeof(struct task_t));
    APP_register_task0(task_list[0]);
    num_tasks++;

    task_list[1] = heapAlloc(sizeof(struct task_t));
    APP_register_task1(task_list[1]);
    num_tasks++;

    /* And start the tasks */
    task_list[0]->_APP_start_task(task_list[0]);
    task_list[1]->_APP_start_task(task_list[1]);
    Platform_enable_interrupts();
}

void OS_uninitialize(void)
{
    OS_log(LOG_INFO, "SEOS shutting down\n");

    Platform_uninitialize();
}

void OS_halt(void)
{
    OS_log(LOG_ERROR, "OS_halt not implemented.");
}

static bool register_subscription(event_type_t event_type,
                                  event_subscription_t subscription)
{
    subscription_bucket_t bucket = subscription_list[event_type];
    if (bucket.items == MAX_SUBSCRIPTIONS) {
        OS_log(LOG_WARN, "Unable to subscribe.  Max subscriptions reached.");
        return false;
    }
    bucket.subscriptions[bucket.items] = subscription;
    bucket.items++;
    return true;
}

/* Will return NULL if no producer is registered. */
static task_t *get_producer_task_for_event(event_type_t event_type)
{
    return subscription_list[event_type].producer;
}

bool OS_event_subscribe(struct task_t *task, event_type_t event_type,
                        nanosec_t latency_ns, nanosec_t jitter_ns,
                        nanosec_t drift_ns, nanotime_t period)
{
    bool success;
    event_subscription_t event_subscription = {latency_ns, jitter_ns,
        drift_ns, period};
    success = register_subscription(event_type, event_subscription);
    task_t *producer = get_producer_task_for_event(event_type);
    if (producer != NULL) {
        task_wakeup_t task_wakeup = {event_type, EVENT_FLAG_PRODUCER_ENABLE,
            producer, nanotime_add(OS_get_time(), period)};
        OS_task_enqueue(task_wakeup);
    } else {
        OS_log(LOG_WARN, "No producer for subscribed event.\n");
        success = false;
    }
    return success;
}

void OS_scheduler(void)
{
    struct task_wakeup_t task_wakeup;
    running = true;

    while(running) {

        /* See if there are any events */
        while (OS_task_queue_empty()) {
            /* Nothing else to do, so execute idle task */
            idle_task._APP_handle_event(&idle_task, EVENT_NULL);
        }
        /* Dequeue next event */
        task_wakeup = OS_task_dequeue();

        if (task_wakeup.event_type == EVENT_HALT) {
            /* Stop the scheduler */
            running = false;
            continue;
        }

        if (task_wakeup.task == NULL) {
            OS_log(LOG_ERROR, "NULL tasks should not be dequeued.");
            continue;
        }

        /* Call the scheduled task. */
        task_wakeup.task->_APP_handle_event(task_wakeup.task, task_wakeup.event_type);
    }
}

/*
 * Event queue
 */
//TODO: lock this
//TODO: swap single event queue out for multiple event queues
void OS_event_data_enqueue(event_data_t event, event_type_t event_type)
{
}

//TODO: lock this
//TODO: swap single event queue out for multiple event queues
event_data_t OS_event_data_dequeue(event_type_t event_type)
{
    event_data_t empty_data = {{0}};
    return empty_data;
}

//TODO: lock this
void OS_task_enqueue(task_wakeup_t task_wakeup)
{
    task_wakeup_queue[task_wakeup_queue_tail] = task_wakeup;
    task_wakeup_queue_tail = (task_wakeup_queue_tail + 1) % TASK_WAKEUP_QUEUE_SIZE;
}

//TODO: lock this
task_wakeup_t OS_task_dequeue(void)
{
    task_wakeup_t task_wakeup = task_wakeup_queue[task_wakeup_queue_head];
    task_wakeup_t empty_task_wakeup = {0};
    task_wakeup_queue[task_wakeup_queue_head] = empty_task_wakeup;
    task_wakeup_queue_head = (task_wakeup_queue_head + 1) %
        TASK_WAKEUP_QUEUE_SIZE;
    return task_wakeup;
}

bool OS_task_queue_empty(void)
{
    return (task_wakeup_queue_head == task_wakeup_queue_tail);
}

/*
 * TODO: revisit interrupts
 */
void OS_interrupt_handler(enum interrupt interrupt, interrupt_handler_t handler)
{
    if (interrupt < INTERRUPT_MAX) {
        interrupt_handler[interrupt] = handler;
    }
}

void OS_interrupt(enum interrupt interrupt, void *data, unsigned len)
{
    /* call registered handler for this interrupt */
    if (interrupt_handler[interrupt]) {
        interrupt_handler[interrupt](data, len);
    }
}

/*
 * Timer
 */
bool OS_add_timer_one_shot(task_t *task, nanotime_t delay,
                                     nanosec_t max_drift_ns,
                                     nanosec_t max_jitter_ns)
{
    bool success;
    bool one_shot = true;

    /* Add the timer to the current set of timers */
    success = Timer_insert(task, delay, 0, max_drift_ns, one_shot);
    return success;
}

bool OS_add_timer_periodic(struct task_t *task,
                                     nanotime_t period,
                                     nanosec_t max_jitter_ns,
                                     nanosec_t max_drift_ns)
{
    bool success;
    bool one_shot = false;

    /* Add the timer to the current set of timers */
    success = Timer_insert(task, period, max_jitter_ns, max_drift_ns,
                          one_shot);

    return success;
}

void OS_cancel_task_timers(struct task_t *task)
{
    Timer_clear_timers_for_task(task);
}

/*
 * idle task
 *
 * this doesn't do much except tell the platform to sleep
 * when there's nothing else to do. when the platform wakes
 * again, the scheduler will need to figure out what
 * happened and schedule another task.
 */
void OS_idle_start_task(struct task_t *task)
{
}

void OS_idle_end_task(struct task_t *task)
{
}

bool OS_idle_handle_event(struct task_t *task, event_type_t event)
{
    /* nothing to do, so sleep the platform to save power */
    Platform_sleep();

    return true;
}

void OS_run_tests(void)
{
    //TODO: Enable other linker script
    //    //TODO surround with ifdef TESTBUILD
    //    TestFunc *t = _stests;
    //    for (t++; t != _etests && (*t)(); t++);
    //    if (t != NULL) {
    //        Platform_log("Test failed.\n");
    //    }
}

/* TODO: Enforce logging by level. */
void OS_log(enum log_level_t level, char *str)
{
    Platform_log(str);
}

struct task_t *OS_get_task(char *taskname)
{
    int i;

    for (i = 0; i < TASK_LIST_SIZE; i++) {
        if (task_list[i] != NULL) {
            if (strcmp(taskname, task_list[i]->name) == 0) {
                return task_list[i];
            }
        }
    }
    return NULL;
}

/* Return number of system ticks since system has been running. */
unsigned OS_get_systick(void)
{
    Platform_get_systick();
    return 0;
}


/*
 * Returns the number of seconds and nanoseconds that have transpired
 * since Epoch (January 1, 1970).
 */
//TODO: Set to RTC
struct nanotime_t OS_get_time(void)
{
    return timer_time;
}
