/*
 * Sample SEOS application
 */

#include <stdlib.h>
#include <string.h>

#include <seos.h>
#include <accelerometer.h>

static void start_task(struct task_t *task)
{
    strcpy(task->name, "test_app0");
    nanotime_t period = {1, 0};
    osEventSubscribe(task, EVENT_SENSOR, 100000, 100000, 100000, period);
    osAddTimerPeriodic(task, period, 100000, 10000);
}

static void end_task(struct task_t *task)
{
    //osUnsubscribeToSensor(task, SENSORTYPEACCELEROMETER);
}

static bool handle_event(struct task_t *task, event_type_t event_type)
{
//    const char *sensorLog = "Test app 0: sensor event received!";
    const char *timerLog = "Test app 0: timer event received!";

    switch(event_type) {
    case EVENT_TIMER:
        osLog(LOG_DEBUG, (void *)timerLog);

        break;

    case EVENT_SENSOR:
        //TODO: Register sensor ISRs in a data struct and call here.
        break;

    case EVENT_RESET:
        // Tells the OS that we're ready to exit
        return false;

    default:
        break;
    }

    return true;
}

void APP_register_task0(struct task_t *task)
{
    task->_APP_start_task = start_task;
    task->_APP_end_task = end_task;
    task->_APP_handle_event = handle_event;
}
