#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#define EVT_APP_START 0x0000300

// These enum and struc delcaration should be done in sensor.h and share by
// all sensors. The event number should be assigned according to certain rules.

enum sensorEvents {
    NO_EVT = -1,
    EVT_SPI_DONE = EVT_APP_START + 1,
    ACC_ACTIVATE,
    GYR_ACTIVATE,
    MAG_ACTIVATE,
    ACC_CONFIG,
    GYR_CONFIG,
    MAG_CONFIG,
    EVT_SENSOR_INTERRUPT_1,
    EVT_SENSOR_INTERRUPT_2,
    ACC_CALIBRATE,
    GYR_CALIBRATE,
    NUM_OF_EVT
};

typedef struct activateStat {
    bool active[2];
} activateStat_t;

typedef struct configStat {
    uint64_t latency_ns_wakeup;
    uint64_t latency_ns;
    uint32_t range;
    uint32_t rate;
} configStat_t;

struct activateCmd
{
    uint32_t evtType;
    activateStat_t data;
} __attribute__((packed));

struct configCmd
{
    uint32_t evtType;
    configStat_t data;
} __attribute__((packed));

struct calibrateCmd
{
    uint32_t evtType;
} __attribute__((packed));


int main(int argc, char *argv[])
{
    struct activateCmd mActivateCmd;
    struct configCmd mConfigCmd;
    struct calibrateCmd mCalibrateCmd;
    int fd;
    char wakeup;

    if (argc < 3) {
        printf("usage: %s <action> <sensor> <data>\n", argv[0]);
        printf("       action: activate|config|calibrate\n");
        printf("       sensor: accel|gyro|mag|als|prox\n");
        printf("       data: activate: true|false\n");
        printf("             config: <rate in Hz> <latency in u-sec>\n");
        printf("             calibrate: [N.A.]\n");

        return 1;
    }


    if (strcmp(argv[1], "activate") == 0) {
        if (argc != 4) {
            printf("Wrong arg number\n");
            return 1;
        }
        if (strcmp(argv[3], "true") == 0)
            mActivateCmd.data.active[0] = true;
        else if (strcmp(argv[3], "false") == 0) {
            mActivateCmd.data.active[0] = false;
        } else {
            printf("Unsupported data: %s For action: %s\n", argv[3], argv[1]);
            return 1;
        }
        mActivateCmd.data.active[0] ;
        if (strcmp(argv[2], "accel") == 0) {
            mActivateCmd.evtType = ACC_ACTIVATE;
        } else if (strcmp(argv[2], "gyro") == 0) {
            mActivateCmd.evtType = GYR_ACTIVATE;
        } else if (strcmp(argv[2], "mag") == 0) {
            mActivateCmd.evtType = MAG_ACTIVATE;
        } else if (strcmp(argv[2], "als") == 0) {
            mActivateCmd.evtType = MAG_ACTIVATE;
        } else if (strcmp(argv[2], "prox") == 0) {
            mActivateCmd.evtType = MAG_ACTIVATE;
        } else {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else if (strcmp(argv[1], "config") == 0) {
        if (argc != 5) {
            printf("Wrong arg number\n");
            return 1;
        }
        mConfigCmd.data.rate = atoi(argv[3]);
        mConfigCmd.data.latency_ns = atoi(argv[4]) * 1000ull;
        if (strcmp(argv[2], "accel") == 0) {
            mConfigCmd.evtType = ACC_CONFIG;
        } else if (strcmp(argv[2], "gyro") == 0) {
            mConfigCmd.evtType = GYR_CONFIG;
        } else if (strcmp(argv[2], "mag") == 0) {
            mConfigCmd.evtType = MAG_CONFIG;
        } else {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else if (strcmp(argv[1], "calibrate") == 0) {
        if (argc != 3) {
            printf("Wrong arg number\n");
            return 1;
        }
        if (strcmp(argv[2], "accel") == 0) {
            mCalibrateCmd.evtType = ACC_CALIBRATE;
        } else if (strcmp(argv[2], "gyro") == 0) {
            mCalibrateCmd.evtType = GYR_CALIBRATE;
        } else {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else {
        printf("Unsupported action: %s\n", argv[1]);
        return 1;
    }

    wakeup = '0';
    fd = open("/sys/class/nanohub/nanohub/wakeup", O_RDWR);
    write(fd, &wakeup, sizeof(wakeup));
    close(fd);

    fd = open("/dev/nanohub", O_RDWR);
    if (strcmp(argv[1], "calibrate") == 0) {
        write(fd, &mCalibrateCmd, sizeof(mCalibrateCmd));
    } else if (strcmp(argv[1], "activate") == 0) {
        write(fd, &mActivateCmd, sizeof(mActivateCmd));
    } else if (strcmp(argv[1], "config") == 0) {
        write(fd, &mConfigCmd, sizeof(mConfigCmd));
    }
    close(fd);

    wakeup = '1';
    fd = open("/sys/class/nanohub/nanohub/wakeup", O_RDWR);
    write(fd, &wakeup, sizeof(wakeup));
    close(fd);


    return 0;
}

