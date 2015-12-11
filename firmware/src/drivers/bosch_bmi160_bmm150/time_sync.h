#ifndef TIME_H_

#define TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

void time_init();

bool sensortime_to_rtc_time(uint64_t sensor_time, uint64_t *rtc_time_ns);

void map_sensortime_to_rtc_time(uint64_t sensor_time, uint64_t rtc_time_ns);
void invalidate_sensortime_to_rtc_time();
void minimize_sensortime_history();

#ifdef __cplusplus
}
#endif

#endif  // TIME_H_
