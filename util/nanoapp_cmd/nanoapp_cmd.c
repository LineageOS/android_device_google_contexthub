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

#include <android/log.h>
#include <assert.h>
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
#include <signal.h>
#include <inttypes.h>

#define SENSOR_RATE_ONCHANGE    0xFFFFFF01UL
#define SENSOR_RATE_ONESHOT     0xFFFFFF02UL
#define SENSOR_HZ(_hz)          ((uint32_t)((_hz) * 1024.0f))
#define MAX_INSTALL_CNT         8

enum ConfigCmds
{
    CONFIG_CMD_DISABLE      = 0,
    CONFIG_CMD_ENABLE       = 1,
    CONFIG_CMD_FLUSH        = 2,
    CONFIG_CMD_CFG_DATA     = 3,
    CONFIG_CMD_CALIBRATE    = 4,
};

struct ConfigCmd
{
    uint32_t evtType;
    uint64_t latency;
    uint32_t rate;
    uint8_t sensorType;
    uint8_t cmd;
    uint16_t flags;
} __attribute__((packed));

struct AppInfo
{
    uint32_t num;
    uint64_t id;
    uint32_t version;
    uint32_t size;
};

static int setType(struct ConfigCmd *cmd, char *sensor)
{
    if (strcmp(sensor, "accel") == 0) {
        cmd->sensorType = SENS_TYPE_ACCEL;
    } else if (strcmp(sensor, "gyro") == 0) {
        cmd->sensorType = SENS_TYPE_GYRO;
    } else if (strcmp(sensor, "mag") == 0) {
        cmd->sensorType = SENS_TYPE_MAG;
    } else if (strcmp(sensor, "uncal_gyro") == 0) {
        cmd->sensorType = SENS_TYPE_GYRO;
    } else if (strcmp(sensor, "uncal_mag") == 0) {
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
    } else if (strcmp(sensor, "gravity") == 0) {
        cmd->sensorType = SENS_TYPE_GRAVITY;
    } else if (strcmp(sensor, "geomag") == 0) {
        cmd->sensorType = SENS_TYPE_GEO_MAG_ROT_VEC;
    } else if (strcmp(sensor, "linear_acc") == 0) {
        cmd->sensorType = SENS_TYPE_LINEAR_ACCEL;
    } else if (strcmp(sensor, "rotation") == 0) {
        cmd->sensorType = SENS_TYPE_ROTATION_VECTOR;
    } else if (strcmp(sensor, "game") == 0) {
        cmd->sensorType = SENS_TYPE_GAME_ROT_VECTOR;
    } else if (strcmp(sensor, "win_orien") == 0) {
        cmd->sensorType = SENS_TYPE_WIN_ORIENTATION;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "tilt") == 0) {
        cmd->sensorType = SENS_TYPE_TILT;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "step_det") == 0) {
        cmd->sensorType = SENS_TYPE_STEP_DETECT;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "step_cnt") == 0) {
        cmd->sensorType = SENS_TYPE_STEP_COUNT;
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
    } else if (strcmp(sensor, "sigmo") == 0) {
        cmd->sensorType = SENS_TYPE_SIG_MOTION;
        cmd->rate = SENSOR_RATE_ONESHOT;
    } else if (strcmp(sensor, "gesture") == 0) {
        cmd->sensorType = SENS_TYPE_GESTURE;
        cmd->rate = SENSOR_RATE_ONESHOT;
    } else if (strcmp(sensor, "hall") == 0) {
        cmd->sensorType = SENS_TYPE_HALL;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "vsync") == 0) {
        cmd->sensorType = SENS_TYPE_VSYNC;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "activity") == 0) {
        cmd->sensorType = SENS_TYPE_ACTIVITY;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else if (strcmp(sensor, "twist") == 0) {
        cmd->sensorType = SENS_TYPE_DOUBLE_TWIST;
        cmd->rate = SENSOR_RATE_ONCHANGE;
    } else {
        return 1;
    }

    return 0;
}

bool drain = false;
bool stop = false;
char *buf;
int nread, buf_size = 2048;
struct AppInfo apps[32];
uint8_t appCount = 0;
char appsToInstall[MAX_INSTALL_CNT][32];
uint8_t installCnt = 0;

void sig_handle(__attribute__((unused)) int sig)
{
    assert(sig == SIGINT);
    printf("Terminating...\n");
    stop = true;
}

void parseInstalledAppInfo()
{
    FILE *fp;
    char *line = NULL;
    size_t len;
    ssize_t numRead;

    fp = fopen("/sys/class/nanohub/nanohub/app_info", "r");
    if (fp == NULL) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to open app_info!");
        printf("Failed to open app_info!\n");
        return;
    }

    while ((numRead = getline(&line, &len, fp)) != -1) {
        struct AppInfo *currApp = &apps[appCount++];
        sscanf(line, "app: %d id: %" PRIx64 " ver: %d size: %d\n", &currApp->num, &currApp->id, &currApp->version, &currApp->size);
    }

    fclose(fp);

    if (line)
        free(line);
}

struct AppInfo *findApp(uint64_t appId)
{
    uint8_t i;

    for (i = 0; i < appCount; i++) {
        if (apps[i].id == appId) {
            return &apps[i];
        }
    }

    return NULL;
}

void parseConfigAppInfo()
{
    FILE *fp;
    char *line = NULL;
    size_t len;
    ssize_t numRead;

    fp = fopen("/vendor/firmware/napp_list.cfg", "r");
    if (fp == NULL) {
        __android_log_write(ANDROID_LOG_WARN, "nanoapp_cmd", "Could not find napp_list.cfg!");
        printf("Could not open napp_list.cfg!\n");
        return;
    }

    parseInstalledAppInfo();

    installCnt = 0;
    while (((numRead = getline(&line, &len, fp)) != -1) && (installCnt < MAX_INSTALL_CNT)) {
        uint64_t appId;
        uint32_t appVersion;
        struct AppInfo* installedApp;

        sscanf(line, "%32s %" PRIx64 " %d\n", appsToInstall[installCnt], &appId, &appVersion);

        installedApp = findApp(appId);
        if (!installedApp || (installedApp->version < appVersion)) {
            installCnt++;
        }
    }

    fclose(fp);

    if (line)
        free(line);
}

void downloadNanohub()
{
    int fd;
    char c = '1';

    fd = open("/sys/class/nanohub/nanohub/download_bl", O_WRONLY);
    if (fd < 0) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to open download_bl!");
        printf("Failed to open download_bl!\n");
        return;
    }

    if (write(fd, &c, 1) <= 0) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to write download_bl!");
        printf("Failed to write download_bl!\n");
    }
    close(fd);
}

void downloadApps()
{
    int fd;
    ssize_t ret;
    uint8_t i;

    fd = open("/sys/class/nanohub/nanohub/download_app", O_WRONLY);
    if (fd < 0) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to open download_app!");
        printf("Failed to open download_app!\n");
        return;
    }

    for (i = 0; i < installCnt; i++) {
        printf("Downloading \"%s.napp\"...\n", appsToInstall[i]);
        ret = write(fd, appsToInstall[i], strlen(appsToInstall[i]));
        if (ret < 0) {
            __android_log_print(ANDROID_LOG_WARN, "nanoapp_cmd", "Failed to download %s.napp!\n", appsToInstall[i]);
            printf("Failed to download %s.napp!\n", appsToInstall[i]);
        }
    }

    close(fd);
}

void resetHub()
{
    int fd;
    char c = '1';

    fd = open("/sys/class/nanohub/nanohub/reset", O_WRONLY);
    if (fd < 0) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to open reset sysfs file!");
        printf("Failed to open reset sysfs file!\n");
        return;
    }

    if (write(fd, &c, 1) <= 0) {
        __android_log_write(ANDROID_LOG_ERROR, "nanoapp_cmd", "Failed to write reset sysfs file!");
        printf("Failed to write reset sysfs file!\n");
    }
    close(fd);
}

int main(int argc, char *argv[])
{
    struct ConfigCmd mConfigCmd;
    int fd;
    int ret;

    if (argc < 3 && strcmp(argv[1], "download") != 0) {
        printf("usage: %s <action> <sensor> <data> -d\n", argv[0]);
        printf("       action: config|calibrate|flush|download\n");
        printf("       sensor: accel|(uncal_)gyro|(uncal_)mag|als|prox|baro|temp|orien\n");
        printf("               gravity|geomag|linear_acc|rotation|game\n");
        printf("               win_orien|tilt|step_det|step_cnt|double_tap\n");
        printf("               flat|anymo|nomo|sigmo|gesture|hall|vsync\n");
        printf("               activity|twist\n");
        printf("       data: config: <true|false> <rate in Hz> <latency in u-sec>\n");
        printf("             calibrate: [N.A.]\n");
        printf("             flush: [N.A.]\n");
        printf("       -d: if specified, %s will keep draining /dev/nanohub until cancelled.\n", argv[0]);

        return 1;
    }

    if (strcmp(argv[1], "config") == 0) {
        if (argc != 6 && argc != 7) {
            printf("Wrong arg number\n");
            return 1;
        }
        if (argc == 7) {
            if(strcmp(argv[6], "-d") == 0) {
                drain = true;
            } else {
                printf("Last arg unsupported, ignored.\n");
            }
        }
        if (strcmp(argv[3], "true") == 0)
            mConfigCmd.cmd = CONFIG_CMD_ENABLE;
        else if (strcmp(argv[3], "false") == 0) {
            mConfigCmd.cmd = CONFIG_CMD_DISABLE;
        } else {
            printf("Unsupported data: %s For action: %s\n", argv[3], argv[1]);
            return 1;
        }
        mConfigCmd.evtType = EVT_NO_SENSOR_CONFIG_EVENT;
        mConfigCmd.rate = SENSOR_HZ((float)atoi(argv[4]));
        mConfigCmd.latency = atoi(argv[5]) * 1000ull;
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
        mConfigCmd.cmd = CONFIG_CMD_CALIBRATE;
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
        mConfigCmd.cmd = CONFIG_CMD_FLUSH;
        if (setType(&mConfigCmd, argv[2])) {
            printf("Unsupported sensor: %s For action: %s\n", argv[2], argv[1]);
            return 1;
        }
    } else if (strcmp(argv[1], "download") == 0) {
        if (argc != 2) {
            printf("Wrong arg number\n");
            return 1;
        }
        downloadNanohub();
        parseConfigAppInfo();
        if (installCnt) {
            downloadApps();
            resetHub();
        }
        return 0;
    } else {
        printf("Unsupported action: %s\n", argv[1]);
        return 1;
    }

    fd = open("/dev/nanohub", O_RDWR);
    do {
        ret = write(fd, &mConfigCmd, sizeof(mConfigCmd));
    } while (ret < 0);
    close(fd);

    if (drain) {
        signal(SIGINT, sig_handle);
        fd = open("/dev/nanohub", O_RDONLY);
        while (!stop) {
            (void) read(fd, buf, buf_size);
        }
        close(fd);
    }
    return 0;
}
