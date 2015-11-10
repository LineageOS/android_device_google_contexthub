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

#define SENSOR_RATE_ONCHANGE    0xFFFFFF01UL
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

static int setType(struct ConfigCmd *cmd, char *sensor)
{
    if (strcmp(sensor, "accel") == 0) {
        cmd->sensorType = SENS_TYPE_ACCEL;
    } else if (strcmp(sensor, "gyro") == 0) {
        cmd->sensorType = SENS_TYPE_GYRO;
    } else if (strcmp(sensor, "mag") == 0) {
        cmd->sensorType = SENS_TYPE_MAG;
    } else if (strcmp(sensor, "als") == 0) {
        cmd->sensorType = SENS_TYPE_ALS;
    } else if (strcmp(sensor, "prox") == 0) {
        cmd->sensorType = SENS_TYPE_PROX;
    } else if (strcmp(sensor, "baro") == 0) {
        cmd->sensorType = SENS_TYPE_BARO;
    } else if (strcmp(sensor, "temp") == 0) {
        cmd->sensorType = SENS_TYPE_TEMP;
    } else if (strcmp(sensor, "orien") == 0) {
        cmd->sensorType = SENS_TYPE_ORIENTATION;
    } else if (strcmp(sensor, "win_orien") == 0) {
        cmd->sensorType = SENS_TYPE_WIN_ORIENTATION;
    } else if (strcmp(sensor, "tilt") == 0) {
        cmd->sensorType = SENS_TYPE_TILT;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "step") == 0) {
        cmd->sensorType = SENS_TYPE_STEP_DETECT;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "double_tap") == 0) {
        cmd->sensorType = SENS_TYPE_DOUBLE_TAP;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "flat") == 0) {
        cmd->sensorType = SENS_TYPE_FLAT;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "anymo") == 0) {
        cmd->sensorType = SENS_TYPE_ANY_MOTION;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "nomo") == 0) {
        cmd->sensorType = SENS_TYPE_NO_MOTION;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "hall") == 0) {
        cmd->sensorType = SENS_TYPE_HALL;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "vsync") == 0) {
        cmd->sensorType = SENS_TYPE_VSYNC;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "activity") == 0) {
        cmd->sensorType = SENS_TYPE_ACTIVITY;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else {
        return 1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    struct ConfigCmd mConfigCmd;
    int fd;
    int ret;

    if (argc < 3) {
        printf("usage: %s <action> <sensor> <data>\n", argv[0]);
        printf("       action: config|calibrate|flush\n");
        printf("       sensor: accel|gyro|mag|als|prox|baro|temp|orien|win_orien\n");
        printf("       data: config: <true|false> <rate in Hz> <latency in u-sec>\n");
        printf("             calibrate: [N.A.]\n");
        printf("             flush: [N.A.]\n");

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
        if (setType(&mConfigCmd, argv[2])) {
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
        if (setType(&mConfigCmd, argv[2])) {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else if (strcmp(argv[1], "flush") == 0) {
        if (argc != 3) {
            printf("Wrong arg number\n");
            return 1;
        }
        mConfigCmd.evtType = EVT_NO_SENSOR_CONFIG_EVENT;
        mConfigCmd.rate = 0;
        mConfigCmd.latency = 0;
        mConfigCmd.enable = 0;
        mConfigCmd.flush = 1;
        mConfigCmd.calibrate = 0;
        if (setType(&mConfigCmd, argv[2])) {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else {
        printf("Unsupported action: %s\n", argv[1]);
        return 1;
    }

    fd = open("/dev/nanohub", O_RDWR);
    do {
        ret = write(fd, &mConfigCmd, sizeof(mConfigCmd));
    } while (ret < 0);
    close(fd);

    return 0;
}
