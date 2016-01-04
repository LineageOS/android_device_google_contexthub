/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef EVENTNUMS_H
#define EVENTNUMS_H

/* These define ranges of reserved events */
#define EVT_NO_FIRST_USER_EVENT          0x00000100    //all events lower than this are reserved for the OS. all of them are nondiscardable necessarily!
#define EVT_NO_FIRST_SENSOR_EVENT        0x00000200    //sensor type SENSOR_TYPE_x produces events of type EVT_NO_FIRST_SENSOR_EVENT + SENSOR_TYPE_x for all Google-defined sensors
#define EVT_NO_SENSOR_CONFIG_EVENT       0x00000300    //event to configure sensors
#define EVT_APP_START                    0x00000400    //sent when an app can actually start

/*
 * These events are in private OS-reserved range, and are sent targettedly
 * to one app. This is OK since real OS-reserved internal events will never
 * go to apps, as that region is reserved for them. We thus achieve succesful
 * overloading of the range.
 */

//for all apps
#define EVT_APP_FREE_EVT_DATA            0x000000FF    //sent to an external app when its event has been marked for freeing. Data: struct AppEventFreeData

//for apps that use I2C
#define EVT_APP_I2C_CBK                  0x000000F0    //data pointer points to struct I2cEventData

//for apps that claim to be a sensor
#define EVT_APP_SENSOR_POWER             0x000000EF    //data pointer is not a pointer, it is a bool encoded as (void*)
#define EVT_APP_SENSOR_FW_UPLD           0x000000EE
#define EVT_APP_SENSOR_SET_RATE          0x000000ED    //data pointer points to a "const struct SensorSetRateEvent"
#define EVT_APP_SENSOR_FLUSH             0x000000EC
#define EVT_APP_SENSOR_TRIGGER           0x000000EB
#define EVT_APP_SENSOR_CALIBRATE         0x000000EA

//for timers
#define EVT_APP_TIMER                    0x000000DF

#endif /* EVENTNUMS_H */
