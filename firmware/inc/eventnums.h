#ifndef EVENT_UAPI_H
#define EVENT_UAPI_H

/* These define ranges of reserved events */
#define EVT_APP_FREE_EVT_DATA            0x000000FF    //sent to an external app when its event has been marked for freeing. Data: struct AppEventFreeData
#define EVT_NO_FIRST_USER_EVENT          0x00000100    //all events lower than this are reserved for the OS. all of them are nondiscardable necessarily!
#define EVT_NO_FIRST_SENSOR_EVENT        0x00000200    //sensor type SENSOR_TYPE_x produces events of type EVT_NO_FIRST_SENSOR_EVENT + SENSOR_TYPE_x for all Google-defined sensors
#define EVT_NO_FIRST_SENSOR_CONFIG_EVENT 0x00000300    //events to configure sensors
#define EVT_APP_START                    0x00000400    //sent when an app can actually start

/* These are sensor configuration events for activation, rate, calibration, etc. */
enum SensorConfigEvent {
    EVT_SENSOR_ACC_ACTIVATE = EVT_NO_FIRST_SENSOR_CONFIG_EVENT,
    EVT_SENSOR_GYR_ACTIVATE,
    EVT_SENSOR_MAG_ACTIVATE,
    EVT_SENSOR_ALS_ACTIVATE,
    EVT_SENSOR_PROX_ACTIVATE,
    EVT_SENSOR_ACC_CONFIG,
    EVT_SENSOR_GYR_CONFIG,
    EVT_SENSOR_MAG_CONFIG,
    EVT_SENSOR_ALS_CONFIG,
    EVT_SENSOR_PROX_CONFIG,
    EVT_SENSOR_ACC_CALIBRATE,
    EVT_SENSOR_GYR_CALIBRATE,
};

#endif /* EVENT_UAPI_H */
