/*
 * Sample SEOS application
 */

#include <stdlib.h>
#include <string.h>

#include <seos.h>

static void start_task(struct task_t *task)
{
    strcpy(task->name, "test_app1");

    task->event_mask |= EVENT_TIMER;

    //osSubscribeToSensor(task, SENSORCOMPASS, 1000);

    nanotime_t period = {5, 0};
    osAddTimerPeriodic(task, period, 4000, 4000);
}

static void end_task(struct task_t *task)
{
    //osUnsubscribeToSensor(task, SENSORCOMPASS);
}

static bool handle_event(struct task_t *task, event_type_t event_type)
{
    const char *sensorLog = "Test app 1: sensor event received!";
    const char *timerLog = "Test app 1: timer event received!";

    //timer_handle_t timer_handle;
    //enum sensor_type_t sensor_type;
    //void *sensor_data;

    switch(event_type) {
    case EVENT_TIMER:
        //timer_handle = osGetTimerHandle(event);
        osLog(LOG_DEBUG, (void *)timerLog);

        break;

    case EVENT_SENSOR:
        osLog(LOG_DEBUG, (void *)sensorLog);
        //sensor_type = osGetSensorType(event);
        //sensor_data = osGetSensorData(event);

        //if (sensor_type == SENSOR_COMPASS) {
            /* blah */
        //}
        break;

    case EVENT_RESET:
        // Tells the OS that we're ready to exit
        return false;

    default:
        break;
    }

    return true;
}

APP_INIT(start_task, end_task, handle_event);
