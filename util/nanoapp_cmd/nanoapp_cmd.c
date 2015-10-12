#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <eventnums.h>
#include <sensType.h>

#define SENSOR_HZ(_hz)          ((uint32_t)((_hz) * 1024.0f))

struct ConfigCmd
{
    uint32_t evtType;
    uint64_t latency;
    uint32_t rate;
    uint8_t sensorType;
    uint8_t enable : 1;
    uint8_t flush : 1;
    uint8_t calibrate : 1;
} __attribute__((packed));

int main(int argc, char *argv[])
{
    struct ConfigCmd mConfigCmd;
    int fd;
    char wakeup;

    if (argc < 3) {
        printf("usage: %s <action> <sensor> <data>\n", argv[0]);
        printf("       action: config|calibrate\n");
        printf("       sensor: accel|gyro|mag|als|prox|baro|temp|orien|win_orien\n");
        printf("       data: config: <true|false> <rate in Hz> <latency in u-sec>\n");
        printf("             calibrate: [N.A.]\n");

        return 1;
    }

    if (strcmp(argv[1], "config") == 0) {
        if (argc != 6) {
            printf("Wrong arg number\n");
            return 1;
        }
        if (strcmp(argv[3], "true") == 0)
            mConfigCmd.enable = 1;
        else if (strcmp(argv[3], "false") == 0) {
            mConfigCmd.enable = 0;
        } else {
            printf("Unsupported data: %s For action: %s\n", argv[3], argv[1]);
            return 1;
        }
        mConfigCmd.evtType = EVT_NO_SENSOR_CONFIG_EVENT;
        mConfigCmd.rate = SENSOR_HZ((float)atoi(argv[4]));
        mConfigCmd.latency = atoi(argv[5]) * 1000ull;
        mConfigCmd.flush = 0;
        mConfigCmd.calibrate = 0;
        if (strcmp(argv[2], "accel") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_ACCEL;
        } else if (strcmp(argv[2], "gyro") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_GYRO;
        } else if (strcmp(argv[2], "mag") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_MAG;
        } else if (strcmp(argv[2], "als") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_ALS;
        } else if (strcmp(argv[2], "prox") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_PROX;
        } else if (strcmp(argv[2], "baro") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_BARO;
        } else if (strcmp(argv[2], "temp") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_TEMP;
        } else if (strcmp(argv[2], "orien") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_ORIENTATION;
        } else if (strcmp(argv[2], "win_orien") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_WIN_ORIENTATION;
        } else if (strcmp(argv[2], "step") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_STEP_DETECT;
        } else if (strcmp(argv[2], "double_tap") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_DOUBLE_TAP;
        } else if (strcmp(argv[2], "flat") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_FLAT;
        } else if (strcmp(argv[2], "anymo") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_ANY_MOTION;
        } else if (strcmp(argv[2], "nomo") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_NO_MOTION;
        } else {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else if (strcmp(argv[1], "calibrate") == 0) {
        if (argc != 3) {
            printf("Wrong arg number\n");
            return 1;
        }
        mConfigCmd.evtType = EVT_NO_SENSOR_CONFIG_EVENT;
        mConfigCmd.rate = 0;
        mConfigCmd.latency = 0;
        mConfigCmd.enable = 0;
        mConfigCmd.flush = 0;
        mConfigCmd.calibrate = 1;
        if (strcmp(argv[2], "accel") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_ACCEL;
        } else if (strcmp(argv[2], "gyro") == 0) {
            mConfigCmd.sensorType = SENS_TYPE_GYRO;
        } else {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else {
        printf("Unsupported action: %s\n", argv[1]);
        return 1;
    }

    fd = open("/dev/nanohub", O_RDWR);
    write(fd, &mConfigCmd, sizeof(mConfigCmd));
    close(fd);

    return 0;
}
