#include <cpu/inc/barrier.h>
#include <atomicBitset.h>
#include <sensors.h>
#include <atomic.h>
#include <stdio.h>
#include <slab.h>
#include <seos.h>

#define MAX_INTERNAL_EVENTS       32
#define MAX_CLI_SENS_MATRIX_SZ    64 /* MAX(numClients * numSensors) */

#define SENSOR_RATE_OFF           0x00000000UL /* used in sensor state machine */
#define SENSOR_RATE_POWERING_ON   0xFFFFFFF0UL /* used in sensor state machine */
#define SENSOR_RATE_POWERING_OFF  0xFFFFFFF1UL /* used in sensor state machine */
#define SENSOR_RATE_FW_UPLOADING  0xFFFFFFF2UL /* used in sensor state machine */
#define SENSOR_RATE_IMPOSSIBLE    0xFFFFFFF3UL /* use din rate calc to indicate impossible combinations */

struct Sensor {
    const struct SensorInfo *si;
    uint32_t handle;         /* here 0 means invalid */
    uint32_t currentRate;    /* here 0 means off */
};

struct SensorsInternalEvent {
    uint32_t handle;
    uint32_t value;
};

struct SensorsClientRequest {
    uint32_t handle;
    uint32_t clientId;
    uint32_t rate;
};

static struct Sensor mSensors[MAX_REGISTERED_SENSORS];
ATOMIC_BITSET_DECL(mSensorsUsed, MAX_REGISTERED_SENSORS, static);
static struct SlabAllocator *mInternalEvents;
static struct SlabAllocator *mCliSensMatrix;
static uint32_t mNextSensorHandle;




bool sensorsInit(void)
{
    mInternalEvents = slabAllocatorNew(sizeof(struct SensorsInternalEvent), 4, MAX_INTERNAL_EVENTS);
    if (!mInternalEvents)
        return false;

    mCliSensMatrix = slabAllocatorNew(sizeof(struct SensorsClientRequest), 4, MAX_CLI_SENS_MATRIX_SZ);
    if (mCliSensMatrix)
        return true;

    slabAllocatorDestroy(mInternalEvents);

    return false;
}

static struct Sensor* sensorFindByHandle(uint32_t handle)
{
    uint32_t i;

    for (i = 0; i < MAX_REGISTERED_SENSORS; i++)
        if (mSensors[i].handle == handle)
            return mSensors + i;

    return NULL;
}

uint32_t sensorRegister(const struct SensorInfo *si)
{
    int32_t idx = atomicBitsetFindClearAndSet(mSensorsUsed);
    struct Sensor *s;
    uint32_t handle;

    /* grab a slot */
    if (idx < 0)
        return 0;

    /* grab a handle */
    do {
        handle = atomicAdd(&mNextSensorHandle, 1);
    } while (!handle || sensorFindByHandle(handle)); /* this is safe since nobody else could have "JUST" taken this handle, we'll need to circle around 32bits before that happens */

    /* fill the struct in and mark it valid (by setting handle) */
    s = mSensors + idx;
    s->si = si;
    s->currentRate = 0;
    mem_reorder_barrier();
    s->handle = handle;

    return handle;
}

bool sensorUnregister(uint32_t handle)
{
    struct Sensor *s = sensorFindByHandle(handle);

    if (!s)
        return false;

    /* mark as invalid */
    s->handle = 0;
    mem_reorder_barrier();

    /* free struct */
    atomicBitsetClearBit(mSensorsUsed, mSensors - s);

    return true;
}

static void sensorReconfig(struct Sensor* s, uint32_t newHwRate)
{
    if (s->currentRate == SENSOR_RATE_OFF) {
        /* if it was off or is off, tell it to come on */
        s->currentRate = SENSOR_RATE_POWERING_ON;
        s->si->ops.sensorPower(true);
    }
    else if (s->currentRate == SENSOR_RATE_POWERING_OFF) {
        /* if it was going to be off or is off, tell it to come back on */
        s->currentRate = SENSOR_RATE_POWERING_ON;
    }
    else if (s->currentRate == SENSOR_RATE_POWERING_ON || s->currentRate == SENSOR_RATE_FW_UPLOADING) {
        /* if it is powering on - do nothing - all will be done for us */
    }
    else if (newHwRate) {
        /* simple rate change - > do it, there is nothing we can do if this fails, so we ignore the immediate errors :( */
        (void)s->si->ops.sensorSetRate(newHwRate);
    }
    else {
        /* powering off */
        s->currentRate = SENSOR_RATE_POWERING_OFF;
        s->si->ops.sensorPower(false);
    }
}

static bool sensorCanDoRate(const struct SensorInfo* si, uint32_t rate)
{
    const uint32_t *rateP = si->supportedRates;

    while (*rateP) {
        if (*rateP == rate)
            return true;
        rateP++;
    }

    return false;
}

static uint64_t sensorCalcLeastCommonMultiple(uint64_t a, uint64_t b)
{
    /* range limits of 64-bit will hurt us here in some cases - watch out */
    /* LCM(a, b) = a * b / GCD (a, b) */

    /* calculate gcd usng euclidean algo */
    uint64_t gcd = a > b ? a : b, t1 = a > b ? b : a, t2;

    while (t1) {
        t2 = t1;
        t1 = gcd % t1;
        gcd = t2;
    }

    /* use it */
    return a * b / gcd;
}

static uint32_t sensorCalcHwRate(struct Sensor* s, uint32_t extraReqedRate, uint32_t removedRate)
{
    uint64_t lcm = extraReqedRate;
    uint32_t i;

    for (i = 0; i < MAX_CLI_SENS_MATRIX_SZ; i++) {
        struct SensorsClientRequest *req = slabAllocatorGetNth(mCliSensMatrix, i);

        /* we only care about this sensor's stuff */
        if (!req || req->handle != s->handle)
            continue;

        /* skip an instance of a removed rate if one was given */
        if (req->rate == removedRate) {
            removedRate = 0;
            continue;
        }

        lcm = lcm ? sensorCalcLeastCommonMultiple(lcm, req->rate) : req->rate;
    }

    if (!lcm)   /* no requests -> we can definitely do that */
        return SENSOR_RATE_OFF;

    if (lcm >> 32) /* too much to even try? */
        return SENSOR_RATE_IMPOSSIBLE;

    return sensorCanDoRate(s->si, lcm) ? lcm : SENSOR_RATE_IMPOSSIBLE;
}

static void sensorInternalFwStateChanged(void *evtP)
{
    struct SensorsInternalEvent *evt = (struct SensorsInternalEvent*)evtP;
    struct Sensor* s = sensorFindByHandle(evt->handle);

    if (s) {

        if (!evt->value) {                                        //we failed -> give up
            s->currentRate = SENSOR_RATE_POWERING_OFF;
            s->si->ops.sensorPower(false);
        }
        else if (s->currentRate == SENSOR_RATE_FW_UPLOADING) {    //we're up
            uint32_t desiredRate = sensorCalcHwRate(s, 0, 0);

            s->currentRate = evt->value;
            if (evt->value != desiredRate)
                sensorReconfig(s, desiredRate);
        }
        else if (s->currentRate == SENSOR_RATE_POWERING_OFF) {    //we need to power off
            s->si->ops.sensorPower(false);
        }
    }
    slabAllocatorFree(mInternalEvents, evt);
}

static void sensorInternalPowerStateChanged(void *evtP)
{
    struct SensorsInternalEvent *evt = (struct SensorsInternalEvent*)evtP;
    struct Sensor* s = sensorFindByHandle(evt->handle);

    if (s) {

        if (s->currentRate == SENSOR_RATE_POWERING_ON && evt->value) {          //we're now on - upload firmware
            s->currentRate = SENSOR_RATE_FW_UPLOADING;
            s->si->ops.sensorFirmwareUpload();
        }
        else if (s->currentRate == SENSOR_RATE_POWERING_OFF && !evt->value) {   //we're now off
            s->currentRate = SENSOR_RATE_OFF;
        }
        else if (s->currentRate == SENSOR_RATE_POWERING_ON && !evt->value) {    //we need to power back on
            s->si->ops.sensorPower(true);
        }
        else if (s->currentRate == SENSOR_RATE_POWERING_OFF && evt->value) {    //we need to power back off
            s->si->ops.sensorPower(false);
        }
    }
    slabAllocatorFree(mInternalEvents, evt);
}

static void sensorInternalRateChanged(void *evtP)
{
    struct SensorsInternalEvent *evt = (struct SensorsInternalEvent*)evtP;
    struct Sensor* s = sensorFindByHandle(evt->handle);

    if (s) {
        s->currentRate = evt->value;
    }
    slabAllocatorFree(mInternalEvents, evt);
}

bool sensorSignalInternalEvt(uint32_t handle, uint32_t intEvtNum, uint32_t value)
{
    static const OsDeferCbkF internalEventCallbacks[] = {
        [SENSOR_INTERNAL_EVT_POWER_STATE_CHG] = sensorInternalPowerStateChanged,
        [SENSOR_INTERNAL_EVT_FW_STATE_CHG] = sensorInternalFwStateChanged,
        [SENSOR_INTERNAL_EVT_RATE_CHG] = sensorInternalRateChanged,
    };
    struct SensorsInternalEvent *evt = (struct SensorsInternalEvent*)slabAllocatorAlloc(mInternalEvents);

    if (!evt)
        return false;

    evt->handle = handle;
    evt->value = value;

    if (osDefer(internalEventCallbacks[intEvtNum], evt))
        return true;

    slabAllocatorFree(mInternalEvents, evt);
    return false;
}

const struct SensorInfo* sensorFind(uint32_t sensorType, uint32_t idx)
{
    uint32_t i;

    for (i = 0; i < MAX_REGISTERED_SENSORS; i++)
        if (mSensors[i].handle && mSensors[i].si->sensorType == sensorType && !idx--)
            return mSensors[i].si;

    return NULL;
}

static bool sensorAddRequestor(uint32_t sensorHandle, uint32_t clientId, uint32_t rate)
{
    struct SensorsClientRequest *req = slabAllocatorAlloc(mCliSensMatrix);

    if (!req)
        return false;

    req->handle = sensorHandle;
    req->clientId = clientId;
    mem_reorder_barrier();
    req->rate = rate;

    return true;
}

static bool sensorGetCurRequestorRate(uint32_t sensorHandle, uint32_t clientId, uint32_t *rateP)
{
    uint32_t i;

    for (i = 0; i < MAX_CLI_SENS_MATRIX_SZ; i++) {
        struct SensorsClientRequest *req = slabAllocatorGetNth(mCliSensMatrix, i);

        if (req && req->handle == sensorHandle && req->clientId == clientId) {
            if (rateP)
                *rateP = req->rate;
            return true;
        }
    }

    return false;
}
static bool sensorAmendRequestor(uint32_t sensorHandle, uint32_t clientId, uint32_t newRate)
{
    uint32_t i;

    for (i = 0; i < MAX_CLI_SENS_MATRIX_SZ; i++) {
        struct SensorsClientRequest *req = slabAllocatorGetNth(mCliSensMatrix, i);

        if (req && req->handle == sensorHandle && req->clientId == clientId) {
            req->rate = newRate;
            return true;
        }
    }

    return false;
}

static bool sensorDeleteRequestor(uint32_t sensorHandle, uint32_t clientId)
{
    uint32_t i;

    for (i = 0; i < MAX_CLI_SENS_MATRIX_SZ; i++) {
        struct SensorsClientRequest *req = slabAllocatorGetNth(mCliSensMatrix, i);

        if (req && req->handle == sensorHandle && req->clientId == clientId) {
            req->rate = 0;
            mem_reorder_barrier();
            slabAllocatorFree(mCliSensMatrix, req);
            return true;
        }
    }

    return false;
}

bool sensorRequest(uint32_t clientId, uint32_t sensorHandle, uint32_t rate)
{
    struct Sensor* s = sensorFindByHandle(sensorHandle);
    uint32_t newSensorRate;

    if (!s)
        return false;

    /* verify the rate is possible */
    if (!sensorCanDoRate(s->si, rate) || (newSensorRate = sensorCalcHwRate(s, 0, rate)) == SENSOR_RATE_IMPOSSIBLE)
        return false;

    /* record the request */
    if (!sensorAddRequestor(sensorHandle, clientId, rate))
        return false;

    /* update actual sensor if needed */
    sensorReconfig(s, newSensorRate);
    return true;
}

bool sensorRequestRateChange(uint32_t clientId, uint32_t sensorHandle, uint32_t newRate)
{
    struct Sensor* s = sensorFindByHandle(sensorHandle);
    uint32_t oldRate, newSensorRate;

    if (!s)
        return false;

    /* verify the rate is possible */
    if (!sensorCanDoRate(s->si, newRate))
        return false;

    /* get current rate */
    if (!sensorGetCurRequestorRate(sensorHandle, clientId, &oldRate))
        return false;

    /* verify the new rate is possible given all othe rongoing requests */
    newSensorRate = sensorCalcHwRate(s, oldRate, newRate);
    if (newSensorRate == SENSOR_RATE_IMPOSSIBLE)
        return false;

    /* record the request */
    if (!sensorAmendRequestor(sensorHandle, clientId, newRate))
        return false;

    /* update actual sensor if needed */
    sensorReconfig(s, newSensorRate);
    return true;
}

bool sensorRelease(uint32_t clientId, uint32_t sensorHandle)
{
    struct Sensor* s = sensorFindByHandle(sensorHandle);
    if (!s)
        return false;

    /* record the request */
    if (!sensorDeleteRequestor(sensorHandle, clientId))
        return false;

    /* update actual sensor if needed */
    sensorReconfig(s, sensorCalcHwRate(s, 0, 0));
    return true;
}

bool sensorTriggerOndemand(uint32_t sensorHandle)
{
    struct Sensor* s = sensorFindByHandle(sensorHandle);

    if (!s)
        return false;

    return s->si->ops.sensorTriggerOndemand();
}







