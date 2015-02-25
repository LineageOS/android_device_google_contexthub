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

    //OS_subscribe_to_sensor(task, SENSOR_COMPASS, 1000);

    nanotime_t period = {5, 0};
    OS_add_timer_periodic(task, period, 4000, 4000);
}

static void end_task(struct task_t *task)
{
    //OS_unsubscribe_to_sensor(task, SENSOR_COMPASS);
}

static bool handle_event(struct task_t *task, event_type_t event_type)
{
    const char *sensor_log = "Test app 1: sensor event received!";
    const char *timer_log = "Test app 1: timer event received!";

    //timer_handle_t timer_handle;
    //enum sensor_type_t sensor_type;
    //void *sensor_data;

    switch(event_type) {
    case EVENT_TIMER:
        //timer_handle = OS_get_timer_handle(event);
        OS_log(LOG_DEBUG, (void *)timer_log);

        break;

    case EVENT_SENSOR:
        OS_log(LOG_DEBUG, (void *)sensor_log);
        //sensor_type = OS_get_sensor_type(event);
        //sensor_data = OS_get_sensor_data(event);

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

void APP_register_task1(struct task_t *task)
{
    task->_APP_start_task = start_task;
    task->_APP_end_task = end_task;
    task->_APP_handle_event = handle_event;
}
