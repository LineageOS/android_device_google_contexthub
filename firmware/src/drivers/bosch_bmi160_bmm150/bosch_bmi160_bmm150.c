#include <stdlib.h>
#include <string.h>
#include <timer.h>
#include <sensors.h>
#include <heap.h>
#include <spi.h>
#include <slab.h>
#include <limits.h>
#include <plat/inc/rtc.h>
#include <plat/inc/gpio.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>
#include <gpio.h>
#include <isr.h>
#include <hostIntf.h>
#include <nanohubPacket.h>
#include <variant/inc/variant.h>

#include <seos.h>
#include <accelerometer.h>

#include "mag_cal.h"
#include "time_sync.h"
#include <nanohub_math.h>

#define BMI160_SPI_WRITE         0x00
#define BMI160_SPI_READ          0x80

#define USE_LOW_POWER                   0
#define CALCULATE_WATERMARK_LEVEL       1

#define BMI160_SPI_BUS_ID         1
#define BMI160_SPI_SPEED_HZ       4000000
#define BMI160_SPI_MODE           3

#define BMI160_INT_IRQ            EXTI9_5_IRQn
#define BMI160_INT1_PIN           GPIO_PB(6)
#define BMI160_INT2_PIN           GPIO_PB(7)

#define BMI160_ID                 0xd1

#define BMI160_REG_ID             0x00
#define BMI160_REG_ERR            0x02
#define BMI160_REG_PMU_STATUS     0x03
#define BMI160_REG_DATA_0         0x04
#define BMI160_REG_DATA_1         0x05
#define BMI160_REG_DATA_14        0x12
#define BMI160_REG_SENSORTIME_0   0x18
#define BMI160_REG_STATUS         0x1b
#define BMI160_REG_INT_STATUS_0   0x1c
#define BMI160_REG_INT_STATUS_1   0x1d
#define BMI160_REG_FIFO_LENGTH_0  0x22
#define BMI160_REG_FIFO_DATA      0x24
#define BMI160_REG_ACC_CONF       0x40
#define BMI160_REG_ACC_RANGE      0x41
#define BMI160_REG_GYR_CONF       0x42
#define BMI160_REG_GYR_RANGE      0x43
#define BMI160_REG_MAG_CONF       0x44
#define BMI160_REG_FIFO_DOWNS     0x45
#define BMI160_REG_FIFO_CONFIG_0  0x46
#define BMI160_REG_FIFO_CONFIG_1  0x47
#define BMI160_REG_MAG_IF_0       0x4b
#define BMI160_REG_MAG_IF_1       0x4c
#define BMI160_REG_MAG_IF_2       0x4d
#define BMI160_REG_MAG_IF_3       0x4e
#define BMI160_REG_MAG_IF_4       0x4f
#define BMI160_REG_INT_EN_0       0x50
#define BMI160_REG_INT_EN_1       0x51
#define BMI160_REG_INT_EN_2       0x52
#define BMI160_REG_INT_OUT_CTRL   0x53
#define BMI160_REG_INT_LATCH      0x54
#define BMI160_REG_INT_MAP_0      0x55
#define BMI160_REG_INT_MAP_1      0x56
#define BMI160_REG_INT_MAP_2      0x57
#define BMI160_REG_INT_MOTION_0   0x5f
#define BMI160_REG_INT_MOTION_1   0x60
#define BMI160_REG_INT_MOTION_2   0x61
#define BMI160_REG_INT_MOTION_3   0x62
#define BMI160_REG_INT_TAP_0      0x63
#define BMI160_REG_INT_TAP_1      0x64
#define BMI160_REG_INT_FLAT_0     0x67
#define BMI160_REG_INT_FLAT_1     0x68
#define BMI160_REG_PMU_TRIGGER    0x6C
#define BMI160_REG_FOC_CONF       0x69
#define BMI160_REG_CONF           0x6a
#define BMI160_REG_IF_CONF        0x6b
#define BMI160_REG_SELF_TEST      0x6d
#define BMI160_REG_OFFSET_0       0x71
#define BMI160_REG_OFFSET_3       0x74
#define BMI160_REG_OFFSET_6       0x77
#define BMI160_REG_STEP_CNT_0     0x78
#define BMI160_REG_STEP_CONF_0    0x7a
#define BMI160_REG_STEP_CONF_1    0x7b
#define BMI160_REG_CMD            0x7e
#define BMI160_REG_MAGIC          0x7f

#define BMM150_REG_CTRL_1         0x4b
#define BMM150_REG_CTRL_2         0x4c
#define BMM150_REG_REPXY          0x51
#define BMM150_REG_REPZ           0x52
#define BMM150_REG_DIG_X1         0x5d
#define BMM150_REG_DIG_Y1         0x5e
#define BMM150_REG_DIG_Z4_LSB     0x62
#define BMM150_REG_DIG_Z4_MSB     0x63
#define BMM150_REG_DIG_X2         0x64
#define BMM150_REG_DIG_Y2         0x65
#define BMM150_REG_DIG_Z2_LSB     0x68
#define BMM150_REG_DIG_Z2_MSB     0x69
#define BMM150_REG_DIG_Z1_LSB     0x6a
#define BMM150_REG_DIG_Z1_MSB     0x6b
#define BMM150_REG_DIG_XYZ1_LSB   0x6c
#define BMM150_REG_DIG_XYZ1_MSB   0x6d
#define BMM150_REG_DIG_Z3_LSB     0x6e
#define BMM150_REG_DIG_Z3_MSB     0x6f
#define BMM150_REG_DIG_XY2        0x70
#define BMM150_REG_DIG_XY1        0x71

#define INT_STEP        0x01
#define INT_ANY_MOTION  0x04
#define INT_DOUBLE_TAP  0x10
#define INT_SINGLE_TAP  0x20
#define INT_ORIENT      0x40
#define INT_FLAT        0x80
#define INT_HIGH_G_Z    0x04
#define INT_LOW_G       0x08
#define INT_DATA_RDY    0x10
#define INT_FIFO_FULL   0x20
#define INT_FIFO_WM     0x40
#define INT_NO_MOTION   0x80

#define gSPI    BMI160_SPI_BUS_ID

#define ACCL_INT_LINE EXTI_LINE_P6
#define GYR_INT_LINE EXTI_LINE_P7

#define SPI_WRITE_0(addr, data) spiQueueWrite(addr, data, 2)
#define SPI_WRITE_1(addr, data, delay) spiQueueWrite(addr, data, delay)
#define GET_SPI_WRITE_MACRO(_1,_2,_3,NAME,...) NAME
#define SPI_WRITE(...) GET_SPI_WRITE_MACRO(__VA_ARGS__, SPI_WRITE_1, SPI_WRITE_0)(__VA_ARGS__)

#define SPI_READ_0(addr, size, buf) spiQueueRead(addr, size, buf, 0)
#define SPI_READ_1(addr, size, buf, delay) spiQueueRead(addr, size, buf, delay)
#define GET_SPI_READ_MACRO(_1,_2,_3,_4,NAME,...) NAME
#define SPI_READ(...) GET_SPI_READ_MACRO(__VA_ARGS__, SPI_READ_1, SPI_READ_0)(__VA_ARGS__)


#define EVT_SENSOR_ACC_DATA_RDY sensorGetMyEventType(SENS_TYPE_ACCEL)
#define EVT_SENSOR_GYR_DATA_RDY sensorGetMyEventType(SENS_TYPE_GYRO)
#define EVT_SENSOR_MAG_DATA_RDY sensorGetMyEventType(SENS_TYPE_MAG)
#define EVT_SENSOR_STEP sensorGetMyEventType(SENS_TYPE_STEP_DETECT)
#define EVT_SENSOR_NO_MOTION sensorGetMyEventType(SENS_TYPE_NO_MOTION)
#define EVT_SENSOR_ANY_MOTION sensorGetMyEventType(SENS_TYPE_ANY_MOTION)
#define EVT_SENSOR_FLAT sensorGetMyEventType(SENS_TYPE_FLAT)
#define EVT_SENSOR_DOUBLE_TAP sensorGetMyEventType(SENS_TYPE_DOUBLE_TAP)
#define EVT_SENSOR_STEP_COUNTER sensorGetMyEventType(SENS_TYPE_STEP_COUNT)

#define MAX_NUM_COMMS_EVENT_SAMPLES 15

#define kScale_acc 0.00239501953f  // ACC_range * 9.81f / 32768.0f;
#define kScale_gyr 0.00106472439f  // GYR_range * M_PI / (180.0f * 32768.0f);
#define kScale_mag 0.0625f         // 1.0f / 16.0f;
#define kTimeSyncPeriodNs   100000000ull // sync sensor and RTC time every 100ms
#define kMinTimeIncrementUs 25000ull // min time increment set to 25ms
#define kSensorTimerIntervalUs 39ull   // bmi160 clock increaments every 39000ns

#define ACC_MIN_RATE    5
#define GYR_MIN_RATE    6
#define ACC_MAX_RATE    12
#define GYR_MAX_RATE    13
#define SENSOR_OSR      3
#define OSR_THRESHOULD  8

enum SensorIndex {
    ACC = 0,
    GYR,
    MAG,
    STEP,
    DTAP,
    FLAT,
    ANYMO,
    NOMO,
    STEPCNT,
    NUM_OF_SENSOR,
};

enum SensorEvents {
    NO_EVT = -1,
    EVT_SPI_DONE = EVT_APP_START + 1,
    EVT_SENSOR_INTERRUPT_1,
    EVT_SENSOR_INTERRUPT_2,
    EVT_TIME_SYNC,
};

enum FifoState {
    FIFO_READ_DATA,
    FIFO_DONE
};

enum InitState {
    RESET_BMI160,
    INIT_BMI160,
    INIT_BMM150,
    INIT_ON_CHANGE_SENSORS,
    INIT_DONE,
};

enum CalibrationState {
    CALIBRATION_START,
    CALIBRATION_FOC,
    CALIBRATION_WAIT_FOC_DONE,
    CALIBRATION_SET_OFFSET,
    CALIBRATION_DONE,
    CALIBRATION_TIMEOUT,
};

enum SensorState {
    SENSOR_BOOT,
    SENSOR_VERIFY_ID,
    SENSOR_INITIALIZING,
    SENSOR_IDLE,
    SENSOR_POWERING_UP,
    SENSOR_POWERING_UP_DONE,
    SENSOR_POWERING_DOWN,
    SENSOR_POWERING_DOWN_DONE,
    SENSOR_CONFIG_CHANGING,
    SENSOR_INT_1_HANDLING,
    SENSOR_INT_2_HANDLING,
    SENSOR_CALIBRATING,
    SENSOR_STEP_CNT,
    SENSOR_TIME_SYNC,
};

enum MagConfigState {
    MAG_SET_START,
    MAG_SET_REPXY,
    MAG_SET_REPZ,
    MAG_SET_DIG_X,
    MAG_SET_DIG_Y,
    MAG_SET_DIG_Z,
    MAG_SET_SAVE_DIG,
    MAG_SET_ADDR,
    MAG_SET_FORCE,
    MAG_SET_DATA,
    MAG_SET_DONE
};

struct ConfigStat {
    uint64_t latency;
    uint32_t rate;
    bool enable;
};

struct BMI160Sensor {
    struct ConfigStat pConfig; // pending config status request
    struct TripleAxisDataEvent *data_evt;
    uint32_t handle;
    uint32_t rate;
    uint64_t latency;
    uint64_t prev_rtc_time;
    uint32_t offset[3];
    bool powered; // activate status
    bool configed; // configure status
    bool offset_enable;
    enum SensorIndex idx;
    uint8_t flush;
};

struct BMI160Task {
    uint32_t tid;
    struct SpiMode mode;
    spi_cs_t cs;
    struct SpiDevice *spiDev;
    uint8_t rxBuffer[1024+4+4];
    uint8_t txBuffer[1024+4+4];
    uint8_t pmuBuffer[2];
    uint8_t errBuffer[2];
    struct SpiPacket packets[30];
    int xferCnt;
    struct Gpio *Int1;
    struct Gpio *Int2;
    struct ChainedIsr Isr1;
    struct ChainedIsr Isr2;
    bool Int1_EN;
    bool Int2_EN;
    bool pending_int[2];
    bool pending_config[NUM_OF_SENSOR];
    struct BMI160Sensor sensors[NUM_OF_SENSOR];
    enum FifoState fifo_state;
    enum InitState init_state;
    enum MagConfigState mag_state;
    enum SensorState state;

    struct MagCal moc;
    bool new_mag_bias;

    enum CalibrationState calibration_state;
    uint8_t calibration_timeout_cnt;

    uint32_t total_step_cnt;
    uint32_t last_step_cnt;

    uint8_t interrupt_enable_0;
    uint8_t interrupt_enable_2;

    uint8_t active_oneshot_sensor_cnt;
    uint8_t active_data_sensor_cnt;

    uint8_t acc_downsample;
    uint8_t gyr_downsample;

    bool pending_time_sync;
    uint8_t sensor_time[4];
    uint64_t time_delta[3];
    uint64_t next_delta[3];
    bool pending_delta[3];
    bool fifo_enabled[3];
    uint64_t last_sensortime;
    uint64_t frame_sensortime;
    uint64_t prev_frame_time[3];
};

static uint32_t AccRates[] = {
    SENSOR_HZ(25.0f/8.0f),
    SENSOR_HZ(25.0f/4.0f),
    SENSOR_HZ(25.0f/2.0f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    SENSOR_HZ(400.0f),
    0,
};

static uint32_t GyrRates[] = {
    SENSOR_HZ(25.0f/8.0f),
    SENSOR_HZ(25.0f/4.0f),
    SENSOR_HZ(25.0f/2.0f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    SENSOR_HZ(400.0f),
    0,
};

static uint32_t MagRates[] = {
    SENSOR_HZ(25.0f/8.0f),
    SENSOR_HZ(25.0f/4.0f),
    SENSOR_HZ(25.0f/2.0f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    0,
};

static struct BMI160Task mTask;
static uint16_t mWbufCnt = 0;
static uint8_t mRegCnt = 0;

static uint8_t mRetryLeft = 5;

static struct SlabAllocator *mDataSlab;

static const struct SensorInfo mSensorInfo[NUM_OF_SENSOR] =
{
    {"Accelerometer",      AccRates, SENS_TYPE_ACCEL,       NUM_AXIS_THREE,    NANOHUB_INT_NONWAKEUP, 3000},
    {"Gyroscope",          GyrRates, SENS_TYPE_GYRO,        NUM_AXIS_THREE,    NANOHUB_INT_NONWAKEUP,   20},
    {"Magnetometer",       MagRates, SENS_TYPE_MAG,         NUM_AXIS_THREE,    NANOHUB_INT_NONWAKEUP,   20},
    {"Step Detector",      NULL,     SENS_TYPE_STEP_DETECT, NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,  100},
    {"Double Tap",         NULL,     SENS_TYPE_DOUBLE_TAP,  NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,   20},
    {"Flat",               NULL,     SENS_TYPE_FLAT,        NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,   20},
    {"Any Motion",         NULL,     SENS_TYPE_ANY_MOTION,  NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,   20},
    {"No Motion",          NULL,     SENS_TYPE_NO_MOTION,   NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,   20},
    {"Step Counter",       NULL,     SENS_TYPE_STEP_COUNT,  NUM_AXIS_EMBEDDED, NANOHUB_INT_NONWAKEUP,   20},
};

static void dataEvtFree(void *ptr)
{
    struct TripleAxisDataEvent *ev = (struct TripleAxisDataEvent *)ptr;
    slabAllocatorFree(mDataSlab, ev);
}

static void spiQueueWrite(uint8_t addr, uint8_t data, uint32_t delay)
{
    mTask.packets[mRegCnt].size = 2;
    mTask.packets[mRegCnt].txBuf = &mTask.txBuffer[mWbufCnt];
    mTask.packets[mRegCnt].rxBuf = &mTask.txBuffer[mWbufCnt];
    mTask.packets[mRegCnt].delay = delay * 1000;
    mTask.txBuffer[mWbufCnt++] = BMI160_SPI_WRITE | addr;
    mTask.txBuffer[mWbufCnt++] = data;
    mRegCnt++;
}

/*
 * need to be sure size of buf is larger than read size
 */
static void spiQueueRead(uint8_t addr, size_t size, void *buf, uint32_t delay)
{
    int i;

    if (!buf) {
        osLog(LOG_ERROR, "rx buffer is none\n");
        return;
    }
    mTask.packets[mRegCnt].size = size + 1;
    mTask.packets[mRegCnt].txBuf = &mTask.txBuffer[mWbufCnt];
    mTask.packets[mRegCnt].rxBuf = buf;
    mTask.packets[mRegCnt].delay = delay * 1000;
    mTask.txBuffer[mWbufCnt++] = BMI160_SPI_READ | addr;
    for (i = 0; i < size; i++)
        mTask.txBuffer[mWbufCnt++] = 0xff;
    mRegCnt++;
}

static void spiBatchTxRx(struct SpiMode *mode,
        SpiCbkF callback, void *cookie)
{
    spiMasterRxTx(mTask.spiDev, mTask.cs,
        mTask.packets, mRegCnt, mode, callback, cookie);
    mRegCnt = 0;
    mWbufCnt = 0;
}

static bool bmi160Isr1(struct ChainedIsr *isr)
{
    struct BMI160Task *data = container_of(isr, struct BMI160Task, Isr1);

    if (!extiIsPendingGpio(data->Int1)) {
        return false;
    }

    osEnqueuePrivateEvt(EVT_SENSOR_INTERRUPT_1, data, NULL, mTask.tid);
    extiClearPendingGpio(data->Int1);
    return true;
}

static bool bmi160Isr2(struct ChainedIsr *isr)
{
    struct BMI160Task *data = container_of(isr, struct BMI160Task, Isr2);

    if (!extiIsPendingGpio(data->Int2))
        return false;

    osEnqueuePrivateEvt(EVT_SENSOR_INTERRUPT_2, data, NULL, mTask.tid);
    extiClearPendingGpio(data->Int2);
    return true;
}

static void sensorSpiCallback(void *cookie, int err)
{
    osEnqueuePrivateEvt(EVT_SPI_DONE, cookie, NULL, mTask.tid);
}

static void sensorTimerCallback(uint32_t timerId, void *data)
{
    osEnqueuePrivateEvt(EVT_SPI_DONE, data, NULL, mTask.tid);
}

static void timeSyncCallback(uint32_t timerId, void *data)
{
    osEnqueuePrivateEvt(EVT_TIME_SYNC, data, NULL, mTask.tid);
}

static bool accFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[ACC].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool gyrFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[GYR].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool magFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[MAG].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool stepFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[STEP].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool doubleTapFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[DTAP].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool noMotionFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[NOMO].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool anyMotionFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[ANYMO].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool flatFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[FLAT].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool stepCntFirmwareUpload(void)
{
    sensorSignalInternalEvt(mTask.sensors[STEPCNT].handle,
            SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool enableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    gpioConfigInput(pin, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(pin);
    extiEnableIntGpio(pin, EXTI_TRIGGER_RISING);
    extiChainIsr(BMI160_INT_IRQ, isr);
    return true;
}

static bool disableInterrupt(struct Gpio *pin, struct ChainedIsr *isr)
{
    extiUnchainIsr(BMI160_INT_IRQ, isr);
    extiDisableIntGpio(pin);
    return true;
}

static void magBias(void)
{
    struct TripleAxisDataEvent *evt = mTask.sensors[MAG].data_evt;
    uint8_t idx;

    if (evt == NULL) {
        idx = 0;
        evt = slabAllocatorAlloc(mDataSlab);
        if (evt == NULL) {
            // slab allocation failed
            osLog(LOG_ERROR, "Slab allocation failed for MAG bias event\n");
            return;
        }
        evt->referenceTime = timGetTime();
    } else {
        idx = evt->samples[0].firstSample.numSamples++;
        evt->samples[idx].deltaTime = timGetTime() - mTask.sensors[MAG].prev_rtc_time;
    }
    evt->samples[0].firstSample.biasPresent = 1;
    evt->samples[0].firstSample.biasSample = idx;
    evt->samples[idx].x = mTask.moc.x_bias;
    evt->samples[idx].y = mTask.moc.y_bias;
    evt->samples[idx].z = mTask.moc.z_bias;
}

static void magIfConfig(void)
{

    //SPI_WRITE(BMI160_REG_MAG_CONF, 0x08);

    // Some magic number to magic reg. Don't know what they are.
    SPI_WRITE(BMI160_REG_CMD, 0x37);
    SPI_WRITE(BMI160_REG_CMD, 0x9a);
    SPI_WRITE(BMI160_REG_CMD, 0xc0);
    SPI_WRITE(BMI160_REG_MAGIC, 0x90);
    SPI_WRITE(BMI160_REG_DATA_1, 0x30);
    SPI_WRITE(BMI160_REG_MAGIC, 0x80);

    // Config the MAG I2C device address
    SPI_WRITE(BMI160_REG_MAG_IF_0, 0x20);

    // set mag_manual_enable, mag_offset=0, mag_rd_burst='8 bytes'
    SPI_WRITE(BMI160_REG_MAG_IF_1, 0x83);

    // primary interface: autoconfig, secondary: magnetometer.
    SPI_WRITE(BMI160_REG_IF_CONF, 0x20);

    // set mag power control bit.
    // It seems only to work when excuted twice.
    // XXX: need to further investigate.
    SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_CTRL_1);
    SPI_WRITE(BMI160_REG_MAG_IF_4, 0x01);
    SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_CTRL_1);
    SPI_WRITE(BMI160_REG_MAG_IF_4, 0x01);
}

static void magConfig(void)
{
    switch (mTask.mag_state) {
    case MAG_SET_START:
        magIfConfig();
        mTask.mag_state = MAG_SET_REPXY;
        break;
    case MAG_SET_REPXY:
        // MAG_SET_REPXY and MAG_SET_REPZ case set:
        // regular preset, f_max,ODR ~ 102 Hz
        SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_REPXY);
        SPI_WRITE(BMI160_REG_MAG_IF_4, 9, 1000000);
        mTask.mag_state = MAG_SET_REPZ;
        break;
    case MAG_SET_REPZ:
        SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_REPZ);
        SPI_WRITE(BMI160_REG_MAG_IF_4, 15, 1000000);
        mTask.mag_state = MAG_SET_DIG_X;
        break;
    case MAG_SET_DIG_X:
        // MAG_SET_DIG_X, MAG_SET_DIG_Y and MAG_SET_DIG_Z cases:
        // save the raw offset for MAG data compensation.
        SPI_WRITE(BMI160_REG_MAG_IF_2, BMM150_REG_DIG_X1, 5000);
        SPI_READ(BMI160_REG_DATA_0, 8, mTask.rxBuffer, 1000000);
        mTask.mag_state = MAG_SET_DIG_Y;
        break;
    case MAG_SET_DIG_Y:
        saveDigData(&mTask.moc, &mTask.rxBuffer[1], 0);
        SPI_WRITE(BMI160_REG_MAG_IF_2, BMM150_REG_DIG_X1 + 8, 5000);
        SPI_READ(BMI160_REG_DATA_0, 8, mTask.rxBuffer, 1000000);
        mTask.mag_state = MAG_SET_DIG_Z;
        break;
    case MAG_SET_DIG_Z:
        saveDigData(&mTask.moc, &mTask.rxBuffer[1], 8);
        SPI_WRITE(BMI160_REG_MAG_IF_2, BMM150_REG_DIG_X1 + 16, 5000);
        SPI_READ(BMI160_REG_DATA_0, 8, mTask.rxBuffer, 1000000);
        mTask.mag_state = MAG_SET_SAVE_DIG;
        break;
    case MAG_SET_SAVE_DIG:
        saveDigData(&mTask.moc, &mTask.rxBuffer[1], 16);
        // fall through, no break;
        mTask.mag_state = MAG_SET_FORCE;
    case MAG_SET_FORCE:
        // set MAG mode to "forced". ready to pull data
        SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_CTRL_2);
        SPI_WRITE(BMI160_REG_MAG_IF_4, 0x02);
        mTask.mag_state = MAG_SET_ADDR;
        break;
    case MAG_SET_ADDR:
        // config MAG read data address to the first BMM150 reg at 0x42
        SPI_WRITE(BMI160_REG_MAG_IF_2, 0x42);
        mTask.mag_state = MAG_SET_DATA;
        break;
    case MAG_SET_DATA:
        // clear mag_manual_en.
        SPI_WRITE(BMI160_REG_MAG_IF_1, 0x03, 1000);
        mTask.mag_state = MAG_SET_DONE;
        break;
    default:
        osLog(LOG_ERROR, "Invalid Mag Config status\n");
        break;
    }
}

static void configInt1(bool on)
{
    enum SensorIndex i;
    if (on && mTask.Int1_EN == false) {
        enableInterrupt(mTask.Int1, &mTask.Isr1);
        mTask.Int1_EN = true;
    } else if (!on && mTask.Int1_EN == true) {
        for (i = ACC; i <= MAG; i++) {
            if (mTask.sensors[i].powered)
                return;
        }
        disableInterrupt(mTask.Int1, &mTask.Isr1);
        mTask.Int1_EN = false;
    }
}

static void configInt2(bool on)
{
    enum SensorIndex i;
    if (on && mTask.Int2_EN == false) {
        enableInterrupt(mTask.Int2, &mTask.Isr2);
        mTask.Int2_EN = true;
    } else if (!on && mTask.Int2_EN == true) {
        for (i = MAG + 1; i < NUM_OF_SENSOR; i++) {
            if (mTask.sensors[i].powered)
                return;
        }
        disableInterrupt(mTask.Int2, &mTask.Isr2);
        mTask.Int2_EN = false;
    }
}

static uint8_t calcWaterMark(void)
{
    int i;
    uint64_t min_latency = ULONG_LONG_MAX;
    uint32_t max_rate = 0;
    uint8_t min_water_mark = 6;
    uint8_t max_water_mark = 200;
    uint8_t water_mark;
    uint32_t temp_cnt, total_cnt = 0;
    uint32_t header_cnt = ULONG_MAX;

    for (i = ACC; i <= MAG; i++) {
        if (mTask.sensors[i].configed && mTask.sensors[i].latency != SENSOR_LATENCY_NODATA) {
            min_latency = mTask.sensors[i].latency < min_latency ? mTask.sensors[i].latency : min_latency;
            max_rate = mTask.sensors[i].rate > max_rate ? mTask.sensors[i].rate : max_rate;
        }
    }

    // if max_rate is less than 50Hz, we lower the minimum water mark level
    if (max_rate < SENSOR_HZ(50.0f)) {
        min_water_mark = 3;
    }

    // if any sensor request no batching, we set a minimum watermark
    // of 24 bytes (12 bytes if all rates are below 50Hz).
    if (min_latency == 0) {
        return min_water_mark;
    }

    // each accel and gyro sample are 6 bytes
    // each mag samlpe is 8 bytes
    // the total number of header byte is estimated by the min samples
    // the actual number of header byte may exceed this estimate but it's ok to
    // batch a bit faster.
    for (i = ACC; i <= MAG; i++) {
        if (mTask.sensors[i].configed && mTask.sensors[i].latency != SENSOR_LATENCY_NODATA) {
            temp_cnt = (uint32_t)(min_latency * (mTask.sensors[i].rate / 1024) / 1000000000ull);
            header_cnt = temp_cnt < header_cnt ? temp_cnt : header_cnt;
            total_cnt += temp_cnt * (i == MAG ? 8 : 6);
        }
    }
    total_cnt += header_cnt;
    water_mark = ((total_cnt / 4) < 0xff) ? (total_cnt / 4) : 0xff; // 4 bytes per count in the water_mark register.
    water_mark = water_mark < min_water_mark ? min_water_mark : water_mark;
    water_mark = water_mark > max_water_mark ? max_water_mark : water_mark;

    return water_mark;
}

static void configFifo(bool on)
{
    uint8_t val = 0x12;
    // if ACC is configed, enable ACC bit in fifo_config reg.
    if (mTask.sensors[ACC].configed && mTask.sensors[ACC].latency != SENSOR_LATENCY_NODATA) {
        val |= 0x40;
        mTask.fifo_enabled[ACC] = true;
    } else {
        mTask.fifo_enabled[ACC] = false;
    }

    // if GYR is configed, enable GYR bit in fifo_config reg.
    if (mTask.sensors[GYR].configed && mTask.sensors[GYR].latency != SENSOR_LATENCY_NODATA) {
        val |= 0x80;
        mTask.fifo_enabled[GYR] = true;
    } else {
        mTask.fifo_enabled[GYR] = false;
    }

    // if MAG is configed, enable MAG bit in fifo_config reg.
    if (mTask.sensors[MAG].configed && mTask.sensors[MAG].latency != SENSOR_LATENCY_NODATA) {
        val |= 0x20;
        mTask.fifo_enabled[MAG] = true;
    } else {
        mTask.fifo_enabled[MAG] = false;
    }

    // clear all fifo data;
    mTask.xferCnt = 1024;
    SPI_READ(BMI160_REG_FIFO_DATA, mTask.xferCnt, mTask.rxBuffer);
    // calculate the new water mark level
    SPI_WRITE(BMI160_REG_FIFO_CONFIG_0, calcWaterMark());
    // write the composed byte to fifo_config reg.
    SPI_WRITE(BMI160_REG_FIFO_CONFIG_1, val);
}

static bool accPower(bool on)
{
    osLog(LOG_INFO, "BMI160: accPower: on=%d, state=%d\n", on, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;

            // set ACC power mode to NORMAL
            SPI_WRITE(BMI160_REG_CMD, 0x11, 50000);
        } else {
            mTask.state = SENSOR_POWERING_DOWN;

            // set ACC power mode to SUSPEND
            SPI_WRITE(BMI160_REG_CMD, 0x10, 5000);
            mTask.sensors[ACC].configed = false;
        }
        mTask.sensors[ACC].powered = on;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);
    } else {
        mTask.pending_config[ACC] = true;
        mTask.sensors[ACC].pConfig.enable = on;
    }
    return true;
}

static bool gyrPower(bool on)
{
    osLog(LOG_INFO, "BMI160: gyrPower: on=%d, state=%d\n", on, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;

            // set GYR power mode to NORMAL
            SPI_WRITE(BMI160_REG_CMD, 0x15, 50000);
        } else {
            mTask.state = SENSOR_POWERING_DOWN;

            // set GYR power mode to SUSPEND
            SPI_WRITE(BMI160_REG_CMD, 0x14, 1000);
            mTask.sensors[GYR].configed = false;
        }
        mTask.sensors[GYR].powered = on;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[GYR]);
    } else {
        mTask.pending_config[GYR] = true;
        mTask.sensors[GYR].pConfig.enable = on;
    }
    return true;
}

static bool magPower(bool on)
{
    osLog(LOG_INFO, "BMI160: magPower: on=%d, state=%d\n", on, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;

            // set MAG power mode to NORMAL
            SPI_WRITE(BMI160_REG_CMD, 0x19, 1000);
            mTask.mag_state = MAG_SET_START;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;

            // set MAG power mode to SUSPEND
            SPI_WRITE(BMI160_REG_CMD, 0x18, 1000);
            mTask.sensors[MAG].configed = false;
        }
        mTask.sensors[MAG].powered = on;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[MAG]);
    } else {
        mTask.pending_config[MAG] = true;
        mTask.sensors[MAG].pConfig.enable = on;
    }
    return true;
}

static bool stepPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        // if step counter is powered, no need to change actual config of step
        // detector.
        if (mTask.sensors[STEPCNT].powered) {
            mTask.sensors[STEP].powered = on;
            return true;
        }
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            mTask.interrupt_enable_2 |= 0x08;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            mTask.interrupt_enable_2 &= ~0x08;
            mTask.sensors[STEP].configed = false;
        }
        mTask.sensors[STEP].powered = on;
        SPI_WRITE(BMI160_REG_INT_EN_2, mTask.interrupt_enable_2);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[STEP]);
    } else {
        mTask.pending_config[STEP] = true;
        mTask.sensors[STEP].pConfig.enable = on;
    }
    return true;
}

static bool flatPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            mTask.interrupt_enable_0 |= 0x80;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            mTask.interrupt_enable_0 &= ~0x80;
            mTask.sensors[FLAT].configed = false;
        }
        mTask.sensors[FLAT].powered = on;
        SPI_WRITE(BMI160_REG_INT_EN_0, mTask.interrupt_enable_0);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[FLAT]);
    } else {
        mTask.pending_config[FLAT] = true;
        mTask.sensors[FLAT].pConfig.enable = on;
    }
    return true;
}

static bool doubleTapPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            mTask.interrupt_enable_0 |= 0x10;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            mTask.interrupt_enable_0 &= ~0x10;
            mTask.sensors[DTAP].configed = false;
        }
        mTask.sensors[DTAP].powered = on;
        SPI_WRITE(BMI160_REG_INT_EN_0, mTask.interrupt_enable_0);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[DTAP]);
    } else {
        mTask.pending_config[DTAP] = true;
        mTask.sensors[DTAP].pConfig.enable = on;
    }
    return true;
}

static bool anyMotionPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            mTask.interrupt_enable_0 |= 0x07;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            mTask.interrupt_enable_0 &= ~0x07;
            mTask.sensors[ANYMO].configed = false;
        }
        mTask.sensors[ANYMO].powered = on;
        SPI_WRITE(BMI160_REG_INT_EN_0, mTask.interrupt_enable_0);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ANYMO]);
    } else {
        mTask.pending_config[ANYMO] = true;
        mTask.sensors[ANYMO].pConfig.enable = on;
    }
    return true;
}

static bool noMotionPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            mTask.interrupt_enable_2 |= 0x07;
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            mTask.interrupt_enable_2 &= ~0x07;
            mTask.sensors[NOMO].configed = false;
        }
        mTask.sensors[NOMO].powered = on;
        SPI_WRITE(BMI160_REG_INT_EN_2, mTask.interrupt_enable_2);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[NOMO]);
    } else {
        mTask.pending_config[NOMO] = true;
        mTask.sensors[NOMO].pConfig.enable = on;
    }
    return true;
}

static bool stepCntPower(bool on)
{
    if (mTask.state == SENSOR_IDLE) {
        if (on) {
            mTask.state = SENSOR_POWERING_UP;
            if (!mTask.sensors[STEP].powered) {
                mTask.interrupt_enable_2 |= 0x08;
                SPI_WRITE(BMI160_REG_INT_EN_2, mTask.interrupt_enable_2);
            }
            // set step_cnt_en bit
            SPI_WRITE(BMI160_REG_STEP_CONF_1, 0x08 | 0x03, 1000);
        } else {
            mTask.state = SENSOR_POWERING_DOWN;
            if (!mTask.sensors[STEP].powered) {
                mTask.interrupt_enable_2 &= ~0x08;
                SPI_WRITE(BMI160_REG_INT_EN_2, mTask.interrupt_enable_2);
            }
            // unset step_cnt_en bit
            SPI_WRITE(BMI160_REG_STEP_CONF_1, 0x03);
            mTask.last_step_cnt = 0;
            mTask.sensors[STEPCNT].configed = false;
        }
        mTask.sensors[STEPCNT].powered = on;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[STEPCNT]);
    } else {
        mTask.pending_config[STEPCNT] = true;
        mTask.sensors[STEPCNT].pConfig.enable = on;
    }
    return true;
}

static void updateTimeDelta(uint8_t idx, uint8_t odr)
{
    if (mTask.fifo_enabled[idx]) {
        // wait till control frame to update, if not disabled
        mTask.next_delta[idx] = 1ull << (16 - odr);
        mTask.pending_delta[idx] = true;
    } else {
        mTask.time_delta[idx] = 1ull << (16 - odr);
    }
}

// compute the register value from sensor rate.
static uint8_t computeOdr(uint32_t rate)
{
    uint8_t odr = 0x00;
    switch (rate) {
    // fall through intended to get the correct register value
    case SENSOR_HZ(3200): odr ++;
    case SENSOR_HZ(1600): odr ++;
    case SENSOR_HZ(800): odr ++;
    case SENSOR_HZ(400): odr ++;
    case SENSOR_HZ(200): odr ++;
    case SENSOR_HZ(100): odr ++;
    case SENSOR_HZ(50): odr ++;
    case SENSOR_HZ(25): odr ++;
    case SENSOR_HZ(25.0f/2.0f): odr ++;
    case SENSOR_HZ(25.0f/4.0f): odr ++;
    case SENSOR_HZ(25.0f/8.0f): odr ++;
    case SENSOR_HZ(25.0f/16.0f): odr ++;
    case SENSOR_HZ(25.0f/32.0f): odr ++;
    default:
        return odr;
    }
}

static bool accSetRate(uint32_t rate, uint64_t latency)
{
    int odr, osr = 0;

    osLog(LOG_INFO, "BMI160: accSetRate: rate=%ld, latency=%lld, state=%d\n", rate, latency, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_CONFIG_CHANGING;

        odr = computeOdr(rate);
        if (odr == -1)
            return false;

        updateTimeDelta(ACC, odr);

        // minimum supported rate for ACCEL is 12.5Hz.
        // Anything lower than that shall be acheived by downsampling.
        if (odr < ACC_MIN_RATE) {
            osr = ACC_MIN_RATE - odr;
            odr = ACC_MIN_RATE;
        }

        // for high odrs, oversample to reduce hw latency and downsample
        // to get desired odr
        if (odr > OSR_THRESHOULD) {
            osr = (SENSOR_OSR + odr) > ACC_MAX_RATE ? (ACC_MAX_RATE - odr) : SENSOR_OSR;
            odr += osr;
        }

        mTask.sensors[ACC].rate = rate;
        mTask.sensors[ACC].latency = latency;
        mTask.sensors[ACC].configed = true;
        mTask.acc_downsample = osr;

        // set ACC bandwidth parameter to 2 (bits[4:6])
        // set the rate (bits[0:3])
        SPI_WRITE(BMI160_REG_ACC_CONF, 0x20 | odr);

        // configure down sampling ratio, 0x88 is to specify we are using
        // filtered samples
        SPI_WRITE(BMI160_REG_FIFO_DOWNS, (mTask.acc_downsample << 4) | mTask.gyr_downsample | 0x88);

        // flush the data and configure the fifo
        configFifo(true);

        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);
    } else {
        mTask.pending_config[ACC] = true;
        mTask.sensors[ACC].pConfig.enable = 1;
        mTask.sensors[ACC].pConfig.rate = rate;
        mTask.sensors[ACC].pConfig.latency = latency;
    }
    return true;
}

static bool gyrSetRate(uint32_t rate, uint64_t latency)
{
    int odr, osr = 0;

    osLog(LOG_INFO, "BMI160: gyrSetRate: rate=%ld, latency=%lld, state=%d\n", rate, latency, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_CONFIG_CHANGING;

        odr = computeOdr(rate);
        if (odr == -1)
            return false;

        updateTimeDelta(GYR, odr);

        // minimum supported rate for GYRO is 25.0Hz.
        // Anything lower than that shall be acheived by downsampling.
        if (odr < GYR_MIN_RATE) {
            osr = GYR_MIN_RATE - odr;
            odr = GYR_MIN_RATE;
        }

        // for high odrs, oversample to reduce hw latency and downsample
        // to get desired odr
        if (odr > OSR_THRESHOULD) {
            osr = (SENSOR_OSR + odr) > GYR_MAX_RATE ? (GYR_MAX_RATE - odr) : SENSOR_OSR;
            odr += osr;
        }

        mTask.sensors[GYR].rate = rate;
        mTask.sensors[GYR].latency = latency;
        mTask.sensors[GYR].configed = true;
        mTask.gyr_downsample = osr;

        // set GYR bandwidth parameter to 2 (bits[4:6])
        // set the rate (bits[0:3])
        SPI_WRITE(BMI160_REG_GYR_CONF, 0x20 | odr);

        // configure down sampling ratio, 0x88 is to specify we are using
        // filtered samples
        SPI_WRITE(BMI160_REG_FIFO_DOWNS, (mTask.acc_downsample << 4) | mTask.gyr_downsample | 0x88);

        // flush the data and configure the fifo
        configFifo(true);

        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[GYR]);
    } else {
        mTask.pending_config[GYR] = true;
        mTask.sensors[GYR].pConfig.enable = 1;
        mTask.sensors[GYR].pConfig.rate = rate;
        mTask.sensors[GYR].pConfig.latency = latency;
    }
    return true;
}

static bool magSetRate(uint32_t rate, uint64_t latency)
{
    int odr;

    osLog(LOG_INFO, "BMI160: magSetRate: rate=%ld, latency=%lld, state=%d\n", rate, latency, mTask.state);

    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_CONFIG_CHANGING;

        mTask.sensors[MAG].rate = rate;
        mTask.sensors[MAG].latency = latency;
        mTask.sensors[MAG].configed = true;

        odr = computeOdr(rate);
        if (odr == -1)
            return false;

        updateTimeDelta(MAG, odr);

        // set the rate for MAG
        SPI_WRITE(BMI160_REG_MAG_CONF, odr);
        configFifo(true);

        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[MAG]);
    } else {
        mTask.pending_config[MAG] = true;
        mTask.sensors[MAG].pConfig.enable = 1;
        mTask.sensors[MAG].pConfig.rate = rate;
        mTask.sensors[MAG].pConfig.latency = latency;
    }
    return true;
}

static bool stepSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[STEP].rate = rate;
    mTask.sensors[STEP].latency = latency;
    mTask.sensors[STEP].configed = true;

    sensorSignalInternalEvt(mTask.sensors[STEP].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool flatSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[FLAT].rate = rate;
    mTask.sensors[FLAT].latency = latency;
    mTask.sensors[FLAT].configed = true;

    sensorSignalInternalEvt(mTask.sensors[FLAT].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool doubleTapSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[DTAP].rate = rate;
    mTask.sensors[DTAP].latency = latency;
    mTask.sensors[DTAP].configed = true;

    sensorSignalInternalEvt(mTask.sensors[DTAP].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool anyMotionSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[ANYMO].rate = rate;
    mTask.sensors[ANYMO].latency = latency;
    mTask.sensors[ANYMO].configed = true;

    sensorSignalInternalEvt(mTask.sensors[ANYMO].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);

    return true;
}

static bool noMotionSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[NOMO].rate = rate;
    mTask.sensors[NOMO].latency = latency;
    mTask.sensors[NOMO].configed = true;

    sensorSignalInternalEvt(mTask.sensors[NOMO].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static bool stepCntSetRate(uint32_t rate, uint64_t latency)
{
    mTask.sensors[STEPCNT].rate = rate;
    mTask.sensors[STEPCNT].latency = latency;
    mTask.sensors[STEPCNT].configed = true;

    sensorSignalInternalEvt(mTask.sensors[STEPCNT].handle,
            SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);
    return true;
}

static void sendFlushEvt(void)
{
    while (mTask.sensors[ACC].flush > 0) {
        osEnqueueEvt(EVT_SENSOR_ACC_DATA_RDY, SENSOR_DATA_EVENT_FLUSH, NULL);
        mTask.sensors[ACC].flush--;
    }
    while (mTask.sensors[GYR].flush > 0) {
        osEnqueueEvt(EVT_SENSOR_GYR_DATA_RDY, SENSOR_DATA_EVENT_FLUSH, NULL);
        mTask.sensors[GYR].flush--;
    }
    while (mTask.sensors[MAG].flush > 0) {
        osEnqueueEvt(EVT_SENSOR_MAG_DATA_RDY, SENSOR_DATA_EVENT_FLUSH, NULL);
        mTask.sensors[MAG].flush--;
    }
}

static void int1Evt(void);

static bool accFlush()
{
    mTask.sensors[ACC].flush++;
    int1Evt();
    return true;
}

static bool gyrFlush()
{
    mTask.sensors[GYR].flush++;
    int1Evt();
    return true;
}

static bool magFlush()
{
    mTask.sensors[MAG].flush++;
    int1Evt();
    return true;
}

static bool stepFlush()
{
    return osEnqueueEvt(EVT_SENSOR_STEP, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool flatFlush()
{
    return osEnqueueEvt(EVT_SENSOR_FLAT, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool doubleTapFlush()
{
    return osEnqueueEvt(EVT_SENSOR_DOUBLE_TAP, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool anyMotionFlush()
{
    return osEnqueueEvt(EVT_SENSOR_ANY_MOTION, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool noMotionFlush()
{
    return osEnqueueEvt(EVT_SENSOR_NO_MOTION, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static bool stepCntFlush()
{
    mTask.sensors[STEPCNT].flush++;
    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_STEP_CNT;
        SPI_READ(BMI160_REG_STEP_CNT_0, 2, mTask.rxBuffer);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[STEPCNT]);
    }
    return true;
}

static void sendStepCnt()
{
    union EmbeddedDataPoint step_cnt;
    uint32_t cur_step_cnt;
    cur_step_cnt = (int)(mTask.rxBuffer[1] | (mTask.rxBuffer[2] << 8));

    if (cur_step_cnt != mTask.last_step_cnt) {
        // Check for possible overflow
        if (cur_step_cnt < mTask.last_step_cnt) {
            // Sanity check: if the 16bits hardware counter is already
            // overflowed, total_step_cnt should greater or equal to
            // last_step_cnt. Otherwise, something is corrupted.
            if (mTask.total_step_cnt < mTask.last_step_cnt) {
                osLog(LOG_ERROR, "step count corrupted, start over\n");
                mTask.total_step_cnt = cur_step_cnt;
            } else {
                mTask.total_step_cnt += cur_step_cnt + (0xFFFF - mTask.last_step_cnt);
            }
        } else {
            mTask.total_step_cnt += (cur_step_cnt - mTask.last_step_cnt);
        }
        mTask.last_step_cnt = cur_step_cnt;
        step_cnt.idata = mTask.total_step_cnt;
        osEnqueueEvt(EVT_SENSOR_STEP_COUNTER, step_cnt.vptr, NULL);
    }

    while(mTask.sensors[STEPCNT].flush) {
        osEnqueueEvt(EVT_SENSOR_STEP_COUNTER, SENSOR_DATA_EVENT_FLUSH, NULL);
        mTask.sensors[STEPCNT].flush--;
    }
}

static uint64_t parseSensortime(uint32_t sensor_time24)
{
    uint32_t prev_time24;
    uint32_t kHalf = 1ul << 23;
    uint64_t full;

    prev_time24 = (uint32_t)mTask.last_sensortime & 0xffffff;

    if (mTask.last_sensortime == 0) {
        mTask.last_sensortime = (uint64_t)sensor_time24;
        return (uint64_t)(sensor_time24);
    }

    if (sensor_time24 == prev_time24) {
        return (uint64_t)(mTask.last_sensortime);
    }

    full = (mTask.last_sensortime & ~0xffffffull) | sensor_time24;

    if (((prev_time24 < sensor_time24) && (sensor_time24 - prev_time24) < kHalf)
            || ((prev_time24 > sensor_time24) && (prev_time24 - sensor_time24) > kHalf)) {
        if (full < mTask.last_sensortime) {
            full += 0x1000000ull;
        }
        mTask.last_sensortime = full;
        return mTask.last_sensortime;
    }

    if (full < mTask.last_sensortime) {
        return full;
    }

    return (full -  0x1000000ull);
}

static void flushRawData(void)
{
    int i;
    for (i = ACC; i <= MAG; i++) {
        if (mTask.sensors[i].data_evt) {

            osEnqueueEvt(sensorGetMyEventType(mSensorInfo[i].sensorType), mTask.sensors[i].data_evt, dataEvtFree);
            mTask.sensors[i].data_evt = NULL;
        }
    }
}

static void parseRawData(struct BMI160Sensor *mSensor, int i, float kScale, uint64_t sensorTime)
{
    float x, y, z;
    int16_t raw_x, raw_y, raw_z;
    struct TripleAxisDataPoint *sample;
    uint32_t delta_time;
    uint64_t rtc_time;

    if (!sensortime_to_rtc_time(sensorTime, &rtc_time)) {
        osLog(LOG_ERROR, "bmi160: Failed time syncing\n");
        return;
    }

    if (rtc_time < mSensor->prev_rtc_time + kMinTimeIncrementUs) {
        rtc_time = mSensor->prev_rtc_time + kMinTimeIncrementUs;
    }

    raw_x = (mTask.rxBuffer[i] | mTask.rxBuffer[i+1] << 8);
    raw_y = (mTask.rxBuffer[i+2] | mTask.rxBuffer[i+3] << 8);
    raw_z = (mTask.rxBuffer[i+4] | mTask.rxBuffer[i+5] << 8);

    if (mSensor->idx == MAG) {
        int32_t mag_x = S16_AT(&mTask.rxBuffer[i]) >> 3;
        int32_t mag_y = S16_AT(&mTask.rxBuffer[i+2]) >> 3;
        int32_t mag_z = S16_AT(&mTask.rxBuffer[i+4]) >> 1;
        uint32_t mag_rhall = U16_AT(&mTask.rxBuffer[i+6]) >>2;

        mag_x = bmm150TempCompensateX(&mTask.moc, mag_x, mag_rhall);
        mag_y = bmm150TempCompensateY(&mTask.moc, mag_y, mag_rhall);
        mag_z = bmm150TempCompensateZ(&mTask.moc, mag_z, mag_rhall);

        BMM150_TO_ANDROID_COORDINATE(mag_x, mag_y, mag_z);

        float xi, yi, zi;
        magCalRemoveSoftiron(&mTask.moc,
                (float)mag_x * kScale,
                (float)mag_y * kScale,
                (float)mag_z * kScale,
                &xi, &yi, &zi);

        mTask.new_mag_bias |= magCalUpdate(&mTask.moc, sensorTime, xi, yi, zi);

        magCalRemoveBias(&mTask.moc, xi, yi, zi, &x, &y, &z);
    } else {

        BMI160_TO_ANDROID_COORDINATE(raw_x, raw_y, raw_z);

        x = (float)raw_x * kScale;
        y = (float)raw_y * kScale;
        z = (float)raw_z * kScale;
    }

    if (mSensor->data_evt == NULL) {
        mSensor->data_evt = slabAllocatorAlloc(mDataSlab);
        if (mSensor->data_evt == NULL) {
            // slab allocation failed
            osLog(LOG_ERROR, "bmi160: IMU: Slab allocation failed\n");
            return;
        }
        // delta time for the first sample is sample count
        mSensor->data_evt->samples[0].firstSample.numSamples = 0;
        mSensor->data_evt->referenceTime = rtc_time;
        mSensor->prev_rtc_time = rtc_time;
    }

    if (mSensor->data_evt->samples[0].firstSample.numSamples >= MAX_NUM_COMMS_EVENT_SAMPLES) {
        osLog(LOG_ERROR, "BAD INDEX\n");
        return;
    }

    sample = &mSensor->data_evt->samples[mSensor->data_evt->samples[0].firstSample.numSamples++];

    // the first deltatime is for sample size
    if (mSensor->data_evt->samples[0].firstSample.numSamples > 1) {
        delta_time = rtc_time - mSensor->prev_rtc_time;
        delta_time = delta_time < 0 ? 0 : delta_time;
        sample->deltaTime = delta_time;
        mSensor->prev_rtc_time = rtc_time;
    }

    sample->x = x;
    sample->y = y;
    sample->z = z;

    //osLog(LOG_INFO, "bmi160: x: %d, y: %d, z: %d\n", (int)(1000*x), (int)(1000*y), (int)(1000*z));

    if (mSensor->data_evt->samples[0].firstSample.numSamples == MAX_NUM_COMMS_EVENT_SAMPLES) {
        flushRawData();
    }

}

static void dispatchData(void)
{
    size_t i = 1, j;
    size_t size = mTask.xferCnt;
    int fh_mode, fh_param;
    uint8_t *buf = mTask.rxBuffer;

    uint64_t min_delta = ULONG_LONG_MAX;
    uint32_t sensor_time24;
    uint64_t full_sensor_time;
    uint64_t frame_sensor_time = mTask.frame_sensortime;
    bool observed[3] = {false, false, false};
    uint64_t tmp_frame_time, tmp_time[3];

    mTask.new_mag_bias = false;

    while (size > 0) {
        if (buf[i] == 0x80) {
            // header 0x80 means no more data
            break;
        }

        fh_mode = buf[i] >> 6;
        fh_param = (buf[i] & 0x1f) >> 2;

        i++;
        size--;

        if (fh_mode == 1) {
            // control frame.
            if (fh_param == 1) {
                // sensortime frame
                for (j = ACC; j <= MAG; j++) {
                    if (mTask.sensors[j].configed && mTask.sensors[j].latency != SENSOR_LATENCY_NODATA) {
                        min_delta = min_delta < mTask.time_delta[j] ? min_delta : mTask.time_delta[j];
                    }
                }
                sensor_time24 = buf[i+2] << 16 | buf[i+1] << 8 | buf[i];
                sensor_time24 &= ~(min_delta - 1);

                full_sensor_time = parseSensortime(sensor_time24);
                mTask.frame_sensortime = full_sensor_time;

                for (j = ACC; j <= MAG; j++) {
                    mTask.prev_frame_time[j] = observed[j] ? full_sensor_time : ULONG_LONG_MAX;
                    if (!mTask.sensors[j].configed || mTask.sensors[j].latency == SENSOR_LATENCY_NODATA) {
                        mTask.prev_frame_time[j] = ULONG_LONG_MAX;
                        mTask.pending_delta[j] = false;
                    }
                }
                i += 3;
                size -= 3;
            } else if (fh_param == 2) {
                // fifo_input config frame
                for (j = ACC; j <= MAG; j++) {
                    if (buf[i] & (0x01 << (j << 1)) && mTask.pending_delta[j]) {
                        mTask.pending_delta[j] = false;
                        mTask.time_delta[j] = mTask.next_delta[j];
                    }
                }
                i++;
                size--;
            } else {
                // skip frame, we skip it
                i++;
                size--;
            }
        } else if (fh_mode == 2) {
            tmp_frame_time = 0;
            for (j = ACC; j <= MAG; j++) {
                observed[j] = (fh_param & (1 << j));
                tmp_time[j] = 0;
                if (mTask.prev_frame_time[j] != ULONG_LONG_MAX && observed[j]) {
                    tmp_time[j] = mTask.prev_frame_time[j] + mTask.time_delta[j];
                    tmp_frame_time = (tmp_time[j] > tmp_frame_time) ? tmp_time[j] : tmp_frame_time;
                }
            }

            if (frame_sensor_time == ULONG_LONG_MAX || tmp_frame_time == 0) {
                if (fh_param & 4) {
                    i+=8;
                    size-=8;
                }
                if (fh_param & 2) {
                    i += 6;
                    size -= 6;
                }
                if (fh_param & 1) {
                    i += 6;
                    size -= 6;
                }
                continue;
            }

            // regular frame, dispatch data to each sensor's own fifo
            if (fh_param & 4) { // have mag data
                parseRawData(&mTask.sensors[MAG], i, kScale_mag, tmp_frame_time);
                //consumed[MAG] = true;
                mTask.prev_frame_time[MAG] = tmp_frame_time;
                i += 8;
                size -= 8;
            }
            if (fh_param & 2) { // have gyro data
                parseRawData(&mTask.sensors[GYR], i, kScale_gyr, tmp_frame_time);
                //consumed[GYR] = true;
                mTask.prev_frame_time[GYR] = tmp_frame_time;
                i += 6;
                size -= 6;
            }
            if (fh_param & 1) { // have accel data
                parseRawData(&mTask.sensors[ACC], i, kScale_acc, tmp_frame_time);
                //consumed[ACC] = true;
                mTask.prev_frame_time[ACC] = tmp_frame_time;
                i += 6;
                size -= 6;
            }
            frame_sensor_time = tmp_frame_time;
        }
    }

    if (mTask.new_mag_bias)
        magBias();

    // flush data events.
    flushRawData();
}

/*
 * Read the interrupt type and send corresponding event
 * If it's anymo or double tap, also send a single uint32 to indicate which axies
 * is this interrupt triggered.
 * If it's flat, also send a bit to indicate flat/non-flat position.
 * If it's step detector, check if we need to send the total step count.
 */
static void int2Handling(void)
{
    union EmbeddedDataPoint trigger_axies;
    uint8_t int_status_0 = mTask.rxBuffer[1];
    uint8_t int_status_1 = mTask.rxBuffer[2];
    if (int_status_0 & INT_STEP) {
        if (mTask.sensors[STEP].powered) {
            osLog(LOG_INFO, "BMI160: Detected step\n");
            osEnqueueEvt(EVT_SENSOR_STEP, NULL, NULL);
        }
        if (mTask.sensors[STEPCNT].powered) {
            mTask.state = SENSOR_STEP_CNT;
            SPI_READ(BMI160_REG_STEP_CNT_0, 2, mTask.rxBuffer);
            spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[STEPCNT]);
        }
    }
    if (int_status_0 & INT_ANY_MOTION) {
        // bit [0:2] of INT_STATUS[2] is set when anymo is triggered by x, y or
        // z axies respectively. bit [3] indicates the slope.
        trigger_axies.idata = (mTask.rxBuffer[3] & 0x0f);
        osLog(LOG_INFO, "BMI160: Detected any motion\n");
        osEnqueueEvt(EVT_SENSOR_ANY_MOTION, trigger_axies.vptr, NULL);
    }
    if (int_status_0 & INT_DOUBLE_TAP) {
        // bit [4:6] of INT_STATUS[2] is set when double tap is triggered by
        // x, y or z axies respectively. bit [7] indicates the slope.
        trigger_axies.idata = ((mTask.rxBuffer[3] & 0xf0) >> 4);
        osLog(LOG_INFO, "BMI160: Detected double tap\n");
        osEnqueueEvt(EVT_SENSOR_DOUBLE_TAP, trigger_axies.vptr, NULL);
    }
    if (int_status_0 & INT_FLAT) {
        // bit [7] of INT_STATUS[3] indicates flat/non-flat position
        trigger_axies.idata = ((mTask.rxBuffer[4] & 0x80) >> 7);
        osLog(LOG_INFO, "BMI160: Detected flat\n");
        osEnqueueEvt(EVT_SENSOR_FLAT, trigger_axies.vptr, NULL);
    }
    if (int_status_1 & INT_NO_MOTION) {
        osLog(LOG_INFO, "BMI160: Detected no motion\n");
        osEnqueueEvt(EVT_SENSOR_NO_MOTION, NULL, NULL);
    }
    return;
}

static void int2Evt(void)
{
    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_INT_2_HANDLING;

        // Read the interrupt reg value to determine what interrupts
        SPI_READ(BMI160_REG_INT_STATUS_0, 4, mTask.rxBuffer);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
    } else {
        mTask.pending_int[1] = true;
    }
}

static void int1Handling(void)
{
    switch (mTask.fifo_state) {
    case FIFO_READ_DATA:
        mTask.fifo_state = FIFO_DONE;
        mTask.xferCnt = 1028;
        // read out fifo.
        SPI_READ(BMI160_REG_FIFO_DATA, mTask.xferCnt, mTask.rxBuffer);
        // record the time.
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;
    default:
        osLog(LOG_ERROR, "Incorrect fifo state\n");
        break;
    }
}

static void int1Evt(void)
{
    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_INT_1_HANDLING;
        mTask.fifo_state = FIFO_READ_DATA;
        int1Handling();
    } else {
        mTask.pending_int[0] = true;
    }
}

// bits[6:7] in OFFSET[6] to enable/disable gyro/accel offset.
// bits[0:5] in OFFSET[6] stores the most significant 2 bits of gyro offset at
// its x, y, z axies.
// Calculate the stored gyro offset and compose it with the intended
// enable/disable mode for gyro/accel offset to determine the value for
// OFFSET[6].
static uint8_t offset6Mode(void)
{
    uint8_t mode = 0;
    if (mTask.sensors[GYR].offset_enable)
        mode |= 0x01 << 7;
    if (mTask.sensors[ACC].offset_enable)
        mode |= 0x01 << 6;
    mode |= (mTask.sensors[GYR].offset[2] & (0x03 << 8)) >> 4;
    mode |= (mTask.sensors[GYR].offset[1] & (0x03 << 8)) >> 6;
    mode |= (mTask.sensors[GYR].offset[0] & (0x03 << 8)) >> 8;
    osLog(LOG_INFO, "OFFSET_6_MODE is: %02x\n", mode);
    return mode;
}

static void accCalibrationHandling(void)
{
    switch (mTask.calibration_state) {
    case CALIBRATION_START:
        mTask.calibration_timeout_cnt = 0;

        //if power is off, turn ACC on to NORMAL mode
        if (!mTask.sensors[ACC].powered)
            SPI_WRITE(BMI160_REG_CMD, 0x11);
        mTask.calibration_state = CALIBRATION_FOC;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);
        break;
    case CALIBRATION_FOC:

        // set accel range to +-8g
        SPI_WRITE(BMI160_REG_ACC_RANGE, 0x08);

        // enable accel fast offset compensation,
        // x: 0g, y: 0g, z: 1g
        SPI_WRITE(BMI160_REG_FOC_CONF, ACC_FOC_CONFIG);

        // start calibration
        SPI_WRITE(BMI160_REG_CMD, 0x03, 100);

        // poll the status reg untill the calibration finishes.
        SPI_READ(BMI160_REG_STATUS, 1, mTask.rxBuffer, 100000);

        mTask.calibration_state = CALIBRATION_WAIT_FOC_DONE;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);
        break;
    case CALIBRATION_WAIT_FOC_DONE:

        // if the STATUS REG has bit 3 set, it means calbration is done.
        // otherwise, check back in 50ms later.
        if (mTask.rxBuffer[1] & 0x08) {

            //disable FOC
            SPI_WRITE(BMI160_REG_FOC_CONF, 0x00);

            //read the offset value for accel
            SPI_READ(BMI160_REG_OFFSET_0, 3, mTask.rxBuffer);
            mTask.calibration_state = CALIBRATION_SET_OFFSET;
            osLog(LOG_INFO, "FOC set FINISHED!\n");
        } else {

            // calibration hasn't finished yet, go back to wait for 50ms.
            SPI_READ(BMI160_REG_STATUS, 1, mTask.rxBuffer, 50000);
            mTask.calibration_state = CALIBRATION_WAIT_FOC_DONE;
            mTask.calibration_timeout_cnt ++;
        }
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);

        // if calbration hasn't finished after 10 polling on the STATUS reg,
        // declare timeout.
        if (mTask.calibration_timeout_cnt == 10) {
            mTask.calibration_state = CALIBRATION_TIMEOUT;
        }
        break;
    case CALIBRATION_SET_OFFSET:
        mTask.sensors[ACC].offset[0] = mTask.rxBuffer[1];
        mTask.sensors[ACC].offset[1] = mTask.rxBuffer[2];
        mTask.sensors[ACC].offset[2] = mTask.rxBuffer[3];
        mTask.sensors[ACC].offset_enable = true;
        osLog(LOG_INFO, "ACCELERATION OFFSET is %02x  %02x  %02x\n",
                (unsigned int)mTask.sensors[ACC].offset[0],
                (unsigned int)mTask.sensors[ACC].offset[1],
                (unsigned int)mTask.sensors[ACC].offset[2]);

        // Enable offset compensation for accel
        uint8_t mode = offset6Mode();
        SPI_WRITE(BMI160_REG_OFFSET_6, mode);

        // if ACC was previous off, turn ACC to SUSPEND
        if (!mTask.sensors[ACC].powered)
            SPI_WRITE(BMI160_REG_CMD, 0x12);
        mTask.calibration_state = CALIBRATION_DONE;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask.sensors[ACC]);
        break;
    default:
        osLog(LOG_ERROR, "Invalid calibration state\n");
        break;
    }
}

static bool accCalibration(void)
{
    if (mTask.state == SENSOR_IDLE) {
        if (mTask.sensors[ACC].powered) {
            osLog(LOG_ERROR, "No calibration allowed when sensor is powered\n");
            return false;
        }
        mTask.state = SENSOR_CALIBRATING;
        mTask.calibration_state = CALIBRATION_START;
        accCalibrationHandling();
    } else {
        osLog(LOG_ERROR, "Accel not IDLE, can't perform calibration\n");
        return false;
    }
    return true;
}

static void gyrCalibrationHandling(void)
{
    // gyro calibration not implemented yet. Seems unnecessary.
    return;
}

static bool gyrCalibration(void)
{
    if (mTask.state == SENSOR_IDLE) {
        mTask.state = SENSOR_CALIBRATING;
        mTask.calibration_state = CALIBRATION_START;
        gyrCalibrationHandling();
    } else {
        osLog(LOG_ERROR, "Gyro not IDLE, can't perform calibration\n");
        return false;
    }
    return true;
}

static const struct SensorOps mSensorOps[NUM_OF_SENSOR] =
{
    {accPower, accFirmwareUpload, accSetRate, accFlush, NULL, accCalibration},
    {gyrPower, gyrFirmwareUpload, gyrSetRate, gyrFlush, NULL, gyrCalibration},
    {magPower, magFirmwareUpload, magSetRate, magFlush, NULL, NULL},
    {stepPower, stepFirmwareUpload, stepSetRate, stepFlush, NULL, NULL},
    {doubleTapPower, doubleTapFirmwareUpload, doubleTapSetRate, doubleTapFlush, NULL, NULL},
    {flatPower, flatFirmwareUpload, flatSetRate, flatFlush, NULL, NULL},
    {anyMotionPower, anyMotionFirmwareUpload, anyMotionSetRate, anyMotionFlush, NULL, NULL},
    {noMotionPower, noMotionFirmwareUpload, noMotionSetRate, noMotionFlush, NULL, NULL},
    {stepCntPower, stepCntFirmwareUpload, stepCntSetRate, stepCntFlush, NULL, NULL},
};

static void configEvent(struct BMI160Sensor *mSensor, struct ConfigStat *ConfigData)
{
    int i;

    for (i=0; &mTask.sensors[i] != mSensor; i++) ;

    if (ConfigData->enable == 0 && mSensor->powered)
        mSensorOps[i].sensorPower(false);
    else if (ConfigData->enable == 1 && !mSensor->powered)
        mSensorOps[i].sensorPower(true);
    else
        mSensorOps[i].sensorSetRate(ConfigData->rate, ConfigData->latency);
}

static void timeSyncEvt(void)
{
    if (mTask.active_data_sensor_cnt == 0) {
        return;
    } else if (mTask.state != SENSOR_IDLE) {
        mTask.pending_time_sync = true;
    } else {
        mTask.state = SENSOR_TIME_SYNC;
        SPI_READ(BMI160_REG_SENSORTIME_0, 3, mTask.sensor_time);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
    }
}

static void sensorInit(void)
{

    // read ERR reg value and Power state reg value for debug purpose.
    SPI_READ(BMI160_REG_ERR, 1, mTask.errBuffer);
    SPI_READ(BMI160_REG_PMU_STATUS, 1, mTask.pmuBuffer);

    switch (mTask.init_state) {
    case RESET_BMI160:
        osLog(LOG_INFO, "BMI160: Performing soft reset\n");
        // perform soft reset and wait for 100ms
        SPI_WRITE(BMI160_REG_CMD, 0xb6, 100000);
        // dummy reads after soft reset, wait 100us
        SPI_READ(BMI160_REG_MAGIC, 1, mTask.rxBuffer, 100);

        mTask.init_state = INIT_BMI160;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;

    case INIT_BMI160:
        // Read any pending interrupts to reset them
        SPI_READ(BMI160_REG_INT_STATUS_0, 4, mTask.rxBuffer);

        // disable accel, gyro and mag data in FIFO, enable header, enable time.
        SPI_WRITE(BMI160_REG_FIFO_CONFIG_1, 0x12);

        // set the watermark to 24 byte
        SPI_WRITE(BMI160_REG_FIFO_CONFIG_0, 0x06);

        // FIFO watermark and fifo_full interrupt enabled
        SPI_WRITE(BMI160_REG_INT_EN_0, 0x00);
        SPI_WRITE(BMI160_REG_INT_EN_1, 0x60);
        SPI_WRITE(BMI160_REG_INT_EN_2, 0x00);

        // INT1, INT2 enabled, high-edge (push-pull) triggered.
        SPI_WRITE(BMI160_REG_INT_OUT_CTRL, 0xbb);

        // INT1, INT2 input disabled, interrupt mode: non-latched
        SPI_WRITE(BMI160_REG_INT_LATCH, 0x00);

        // Map data interrupts (e.g., FIFO) to INT1 and physical
        // interrupts (e.g., any motion) to INT2
        SPI_WRITE(BMI160_REG_INT_MAP_0, 0x00);
        SPI_WRITE(BMI160_REG_INT_MAP_1, 0xE1);
        SPI_WRITE(BMI160_REG_INT_MAP_2, 0xFF);

        // Disable PMU_TRIGGER
        SPI_WRITE(BMI160_REG_PMU_TRIGGER, 0x00);

        // tell gyro and accel to NOT use the FOC offset.
        SPI_WRITE(BMI160_REG_OFFSET_6, 0x00);

        // initial range for accel (+-8g) and gyro (+-2000 degree).
        SPI_WRITE(BMI160_REG_ACC_RANGE, 0x08);
        SPI_WRITE(BMI160_REG_GYR_RANGE, 0x00);

        // Reset step counter
        SPI_WRITE(BMI160_REG_CMD, 0xB2, 10000);
        // Reset interrupt
        SPI_WRITE(BMI160_REG_CMD, 0xB1, 10000);
        // Reset fifo
        SPI_WRITE(BMI160_REG_CMD, 0xB0, 10000);

        mTask.init_state = INIT_BMM150;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;

    case INIT_BMM150:
        // set the MAG power to NORMAL mode
        SPI_WRITE(BMI160_REG_CMD, 0x19, 10000);

        // Mag setup magic squence... Don't know what they are doing.
        SPI_WRITE(BMI160_REG_CMD, 0x37);
        SPI_WRITE(BMI160_REG_CMD, 0x9a);
        SPI_WRITE(BMI160_REG_CMD, 0xc0);
        SPI_WRITE(BMI160_REG_MAGIC, 0x90);
        SPI_WRITE(BMI160_REG_DATA_1, 0x30);
        SPI_WRITE(BMI160_REG_MAGIC, 0x80);

        // setup MAG I2C address.
        SPI_WRITE(BMI160_REG_MAG_IF_0, 0x20);

        // set mag_manual_enable, mag_offset=0, mag_rd_burst='0 bytes'
        // Seems need to set burst read size to 0 bytes once at the
        // beginning
        SPI_WRITE(BMI160_REG_MAG_IF_1, 0x80);

        // primary interface: autoconfig, secondary: magnetometer.
        SPI_WRITE(BMI160_REG_IF_CONF, 0x20);

        // Turn on and turn off (toggle once) the power control bit on
        // BMM150
        SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_CTRL_1);
        SPI_WRITE(BMI160_REG_MAG_IF_4, 0x01, 60000);
        SPI_WRITE(BMI160_REG_MAG_IF_3, BMM150_REG_CTRL_1);
        SPI_WRITE(BMI160_REG_MAG_IF_4, 0x00, 60000);
        SPI_WRITE(BMI160_REG_MAG_IF_1, 0x00);

        // set the MAG power to SUSPEND mode
        SPI_WRITE(BMI160_REG_CMD, 0x18, 10000);

        mTask.init_state = INIT_ON_CHANGE_SENSORS;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;

    case INIT_ON_CHANGE_SENSORS:
        // set any_motion duration to 0
        // set no_motion duration to ~5sec
        SPI_WRITE(BMI160_REG_INT_MOTION_0, 0x0c);

        // set any_motion threshould to 5*15.63mg(for 8g range)=78.15mg
        // I use the same value as chinook...
        SPI_WRITE(BMI160_REG_INT_MOTION_1, 0x05);

        // set no_motion threshould to 10*15.63mg (for 8g range)
        // I use the same value as chinook.. Don't know why it is higher than
        // any_motion.
        SPI_WRITE(BMI160_REG_INT_MOTION_2, 0x0A);

        // select no_motion over slow_motion
        // select any_motion over significant motion
        SPI_WRITE(BMI160_REG_INT_MOTION_3, 0x15);

        // int_tap_quiet=30ms, int_tap_shock=75ms, int_tap_dur=150ms
        SPI_WRITE(BMI160_REG_INT_TAP_0, 0x42);

        // int_tap_th = 7 * 250 mg (8-g range)
        SPI_WRITE(BMI160_REG_INT_TAP_1, TAP_THRESHOULD);

        // config step detector
        SPI_WRITE(BMI160_REG_STEP_CONF_0, 0x15);
        SPI_WRITE(BMI160_REG_STEP_CONF_1, 0x03);

        // int_flat_theta = 44.8 deg * (16/64) = 11.2 deg
        SPI_WRITE(BMI160_REG_INT_FLAT_0, 0x10);

        // int_flat_hold_time = (640 msec)
        // int_flat_hy = 44.8 * 4 / 64 = 2.8 deg
        SPI_WRITE(BMI160_REG_INT_FLAT_1, 0x14);

        mTask.init_state = INIT_DONE;
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;

    default:
        osLog(LOG_INFO, "Invalid init_state.\n");
    }
}

static void processPendingEvt(void)
{
    enum SensorIndex i;
    if (mTask.pending_int[0]) {
        osLog(LOG_INFO, "INT1 PENDING!!\n");
        mTask.pending_int[0] = false;
        int1Evt();
        return;
    }
    if (mTask.pending_int[1]) {
        osLog(LOG_INFO, "INT2 PENDING!!\n");
        mTask.pending_int[1] = false;
        int2Evt();
        return;
    }
    if (mTask.pending_time_sync) {
        osLog(LOG_INFO, "Pending Time Sync Evt\n");
        mTask.pending_time_sync = false;
        timeSyncEvt();
        return;
    }

    for (i = ACC; i < NUM_OF_SENSOR; i++) {
        if (mTask.pending_config[i]) {
            osLog(LOG_INFO, "Process pending config event for %s\n", mSensorInfo[i].sensorName);
            mTask.pending_config[i] = false;
            configEvent(&mTask.sensors[i], &mTask.sensors[i].pConfig);
            return;
        }
    }
    if (mTask.sensors[STEPCNT].flush > 0) {
        stepCntFlush();
        return;
    }
}

static void handleSpiDoneEvt(const void* evtData)
{
    struct BMI160Sensor *mSensor;
    uint64_t SensorTime;
    int i;

    switch (mTask.state) {
    case SENSOR_BOOT:
        mTask.state = SENSOR_VERIFY_ID;
        // dummy reads after boot, wait 100us
        SPI_READ(BMI160_REG_MAGIC, 1, mTask.rxBuffer, 100);
        // read the device ID for bmi160
        SPI_READ(BMI160_REG_ID, 1, &mTask.rxBuffer[2]);
        SPI_READ(BMI160_REG_ERR, 1, mTask.errBuffer);
        SPI_READ(BMI160_REG_PMU_STATUS, 1, mTask.pmuBuffer);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, &mTask);
        break;
    case SENSOR_VERIFY_ID:
        if (mTask.rxBuffer[3] != BMI160_ID) {
            mRetryLeft --;
            osLog(LOG_ERROR, " BMI160 failed id match: %02x\n", mTask.rxBuffer[1]);
            if (mRetryLeft == 0)
                break;
            // For some reason the first ID read will fail to get the
            // correct value. need to retry a few times.
            mTask.state = SENSOR_BOOT;
            timTimerSet(100000000, 100, 100, sensorTimerCallback, NULL, true);
            break;
        } else {
            mTask.state = SENSOR_INITIALIZING;
            mTask.init_state = RESET_BMI160;
            sensorInit();
            break;
        }
    case SENSOR_INITIALIZING:

        if (mTask.init_state == INIT_DONE) {
            osLog(LOG_INFO, "Done initialzing, system IDLE\n");
            for (i=0; i<NUM_OF_SENSOR; i++)
                sensorRegisterInitComplete(mTask.sensors[i].handle);
            mTask.state = SENSOR_IDLE;
            // In case other tasks have already requested us before we finish booting up.
            processPendingEvt();
        } else {
            sensorInit();
        }
        osLog(LOG_INFO, "   ERR: %02x, PMU: %02x\n", mTask.errBuffer[1], mTask.pmuBuffer[1]);
        break;
    case SENSOR_POWERING_UP:
        mSensor = (struct BMI160Sensor *)evtData;
        if (mSensor->idx == MAG && mTask.mag_state != MAG_SET_DONE) {
            magConfig();
        } else {
            if (mSensor->idx <= MAG) {
                if (++mTask.active_data_sensor_cnt == 1) {
                    // if this is the first data sensor to enable, we need to
                    // sync the sensor time and rtc time
                    invalidate_sensortime_to_rtc_time();
                    timTimerSet(kTimeSyncPeriodNs, 100, 100, timeSyncCallback, NULL, true);
                } else if (mSensor->idx == GYR) {
                    minimize_sensortime_history();
                }
                configInt1(true);
            } else {
                if (++mTask.active_oneshot_sensor_cnt == 1) {
                    // if this is the first one-shot sensor to enable, we need
                    // to request the accel at 50Hz.
                    sensorRequest(mTask.tid, mTask.sensors[ACC].handle, SENSOR_HZ(50), SENSOR_LATENCY_NODATA);
                }
                configInt2(true);
            }
            mTask.state = SENSOR_POWERING_UP_DONE;
        }
        SPI_READ(BMI160_REG_ERR, 1, mTask.errBuffer);
        SPI_READ(BMI160_REG_PMU_STATUS, 1, mTask.pmuBuffer);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, mSensor);
        break;
    case SENSOR_POWERING_UP_DONE:
        mSensor = (struct BMI160Sensor *)evtData;
        sensorSignalInternalEvt(mSensor->handle,
                SENSOR_INTERNAL_EVT_POWER_STATE_CHG, 1, 0);
        mTask.state = SENSOR_IDLE;
        osLog(LOG_INFO, "Done powering up for %s\n", mSensorInfo[mSensor->idx].sensorName);
        osLog(LOG_INFO, "   NEW POWER STATUS: %02x, ERROR: %02x\n", mTask.pmuBuffer[1], mTask.errBuffer[1]);
        processPendingEvt();
        break;
    case SENSOR_POWERING_DOWN:
        mSensor = (struct BMI160Sensor *)evtData;
        mTask.state = SENSOR_POWERING_DOWN_DONE;
        if (mSensor->idx <= MAG) {
            configInt1(false);
            configFifo(false);
            if (--mTask.active_data_sensor_cnt == 0) {
                // if this is the last data sensor to disable, we need to flush
                // the fifo and invalidate time
                mTask.frame_sensortime = ULONG_LONG_MAX;
                SPI_WRITE(BMI160_REG_CMD, 0xb0);
                for (i = ACC; i <= MAG; i++) {
                    mTask.pending_delta[i] = false;
                    mTask.prev_frame_time[i] = ULONG_LONG_MAX;
                }
            } else if (mSensor->idx == GYR) {
                minimize_sensortime_history();
            }
        } else {
            configInt2(false);
            if (--mTask.active_oneshot_sensor_cnt == 0) {
                // if this is the last one-shot sensor to disable, we need to
                // release the accel.
                sensorRelease(mTask.tid, mTask.sensors[ACC].handle);
            }
        }
        SPI_READ(BMI160_REG_ERR, 1, mTask.errBuffer);
        SPI_READ(BMI160_REG_PMU_STATUS, 1, mTask.pmuBuffer);
        spiBatchTxRx(&mTask.mode, sensorSpiCallback, mSensor);
        break;
    case SENSOR_POWERING_DOWN_DONE:
        mSensor = (struct BMI160Sensor *)evtData;
        sensorSignalInternalEvt(mSensor->handle,
                SENSOR_INTERNAL_EVT_POWER_STATE_CHG, 0, 0);
        mTask.state = SENSOR_IDLE;
        osLog(LOG_INFO, "Done powering down for %s\n", mSensorInfo[mSensor->idx].sensorName);
        osLog(LOG_INFO, "   NEW POWER STATUS: %02x, ERROR: %02x\n", mTask.pmuBuffer[1], mTask.errBuffer[1]);

        if (mTask.active_data_sensor_cnt > 0)
            dispatchData();

        processPendingEvt();
        break;
    case SENSOR_INT_1_HANDLING:
        if (mTask.fifo_state == FIFO_DONE) {

            dispatchData();
            sendFlushEvt();
            mTask.state = SENSOR_IDLE;

            processPendingEvt();
        } else {
            int1Handling();
        }
        break;
    case SENSOR_INT_2_HANDLING:
        int2Handling();
        if (mTask.state == SENSOR_INT_2_HANDLING) {
            mTask.state = SENSOR_IDLE;
            processPendingEvt();
        }
        break;
    case SENSOR_CONFIG_CHANGING:
        mSensor = (struct BMI160Sensor *)evtData;
        sensorSignalInternalEvt(mSensor->handle,
                SENSOR_INTERNAL_EVT_RATE_CHG, mSensor->rate, mSensor->latency);
        mTask.state = SENSOR_IDLE;
        osLog(LOG_INFO, "Done changing config\n");
        dispatchData();
        processPendingEvt();
        break;
    case SENSOR_CALIBRATING:
        mSensor = (struct BMI160Sensor *)evtData;
        if (mTask.calibration_state == CALIBRATION_DONE) {
            osLog(LOG_INFO, "DONE calibration\n");
            mTask.state = SENSOR_IDLE;
            processPendingEvt();
        } else if (mTask.calibration_state == CALIBRATION_TIMEOUT) {
            osLog(LOG_INFO, "Calibration TIMED OUT\n");
            mTask.state = SENSOR_IDLE;
            processPendingEvt();
        } else if (mSensor->idx == ACC) {
            accCalibrationHandling();
        } else if (mSensor->idx == GYR) {
            gyrCalibrationHandling();
        }
        break;
    case SENSOR_STEP_CNT:
        sendStepCnt();
        mTask.state = SENSOR_IDLE;
        processPendingEvt();
        break;
    case SENSOR_TIME_SYNC:
        SensorTime = parseSensortime(mTask.sensor_time[1] | (mTask.sensor_time[2] << 8) | (mTask.sensor_time[3] << 16));
        map_sensortime_to_rtc_time(SensorTime, rtcGetTime());

        if (mTask.active_data_sensor_cnt > 0)
            timTimerSet(kTimeSyncPeriodNs, 100, 100, timeSyncCallback, NULL, true);

        mTask.state = SENSOR_IDLE;
        processPendingEvt();
        break;
    default:
        break;
    }
}

static void handleEvent(uint32_t evtType, const void* evtData)
{
    uint64_t currTime;

    switch (evtType) {
    case EVT_APP_START:
        mTask.state = SENSOR_BOOT;
        osEventUnsubscribe(mTask.tid, EVT_APP_START);

        // wait 100ms for sensor to boot
        currTime = timGetTime();
        if (currTime < 100000000ULL) {
            timTimerSet(100000000 - currTime, 100, 100, sensorTimerCallback, NULL, true);
            break;
        }
        /* We have already been powered on long enough - fall through */
    case EVT_SPI_DONE:
        handleSpiDoneEvt(evtData);
        break;
    case EVT_SENSOR_INTERRUPT_1:
        int1Evt();
        break;
    case EVT_SENSOR_INTERRUPT_2:
        int2Evt();
        break;
    case EVT_TIME_SYNC:
        timeSyncEvt();
    default:
        break;
    }
}

static void initSensorStruct(struct BMI160Sensor *sensor, enum SensorIndex idx)
{
    sensor->idx = idx;
    sensor->powered = false;
    sensor->configed = false;
    sensor->rate = 0;
    sensor->offset[0] = 0;
    sensor->offset[1] = 0;
    sensor->offset[2] = 0;
    sensor->latency = 0;
    sensor->data_evt = NULL;
    sensor->flush = 0;
    sensor->prev_rtc_time = 0;
}

static bool startTask(uint32_t task_id)
{
    osLog(LOG_INFO, "        IMU:  %ld\n", task_id);

    enum SensorIndex i;
    size_t slabSize;

    time_init();

    mTask.tid = task_id;

    mTask.Int1 = gpioRequest(BMI160_INT1_PIN);
    mTask.Isr1.func = bmi160Isr1;
    mTask.Int2 = gpioRequest(BMI160_INT2_PIN);
    mTask.Isr2.func = bmi160Isr2;
    mTask.Int1_EN = false;
    mTask.Int2_EN = false;
    mTask.pending_int[0] = false;
    mTask.pending_int[1] = false;

    mTask.mode.speed = BMI160_SPI_SPEED_HZ;
    mTask.mode.bitsPerWord = 8;
    mTask.mode.cpol = SPI_CPOL_IDLE_HI;
    mTask.mode.cpha = SPI_CPHA_TRAILING_EDGE;
    mTask.mode.nssChange = true;
    mTask.mode.format = SPI_FORMAT_MSB_FIRST;
    mTask.cs = GPIO_PB(12);
    spiMasterRequest(BMI160_SPI_BUS_ID, &mTask.spiDev);

    for (i = ACC; i < NUM_OF_SENSOR; i++) {
        initSensorStruct(&mTask.sensors[i], i);
        mTask.sensors[i].handle = sensorRegister(&mSensorInfo[i], &mSensorOps[i]);
        mTask.pending_config[i] = false;
    }

    osEventSubscribe(mTask.tid, EVT_APP_START);

    initMagCal(&mTask.moc,
            0.0f, 0.0f, 0.0f,      // bias x, y, z
            1.0f, 0.0f, 0.0f,      // c00, c01, c02
            0.0f, 1.0f, 0.0f,      // c10, c11, c12
            0.0f, 0.0f, 1.0f);     // c20, c21, c22

    slabSize = sizeof(struct TripleAxisDataEvent) +
        MAX_NUM_COMMS_EVENT_SAMPLES * sizeof(struct TripleAxisDataPoint);

    // each event has 15 samples, with 7 bytes per sample from the fifo.
    // the fifo size is 1K.
    // 20 slabs because some slabs may only hold 1-2 samples.
    // XXX: this consumes too much memeory, need to optimize
    mDataSlab = slabAllocatorNew(slabSize, 4, 20);
    if (!mDataSlab) {
        osLog(LOG_INFO, "Slab allocation failed\n");
        return false;
    }

    mTask.interrupt_enable_0 = 0x00;
    mTask.interrupt_enable_2 = 0x00;

    // initialize the last bmi160 time to be ULONG_MAX, so that we know it's
    // not valid yet.
    mTask.last_sensortime = 0;
    mTask.frame_sensortime = ULONG_LONG_MAX;

    return true;
}

static void endTask(void)
{
    destroy_mag_cal(&mTask.moc);
    slabAllocatorDestroy(mDataSlab);
    spiMasterRelease(mTask.spiDev);
    gpioRelease(mTask.Int1);
    gpioRelease(mTask.Int2);
}

INTERNAL_APP_INIT(APP_ID_MAKE(APP_ID_VENDOR_GOOGLE, 2), startTask, endTask, handleEvent);
