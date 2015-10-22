#include <plat/inc/gpio.h>
#include <plat/inc/usart.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/pwr.h>
#include <plat/inc/rtc.h>
#include <plat/inc/plat.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <platform.h>
#include <seos.h>
#include <heap.h>
#include <timer.h>
#include <usart.h>
#include <gpio.h>
#include <mpu.h>
#include <cpu.h>
#include <hostIntf.h>
#include <atomic.h>
#include <nanohubPacket.h>
#include <variant/inc/variant.h>


struct StmDbg {
    volatile uint32_t IDCODE;
    volatile uint32_t CR;
    volatile uint32_t APB1FZ;
    volatile uint32_t APB2FZ;
};

struct StmTim {

    volatile uint16_t CR1;
    uint8_t unused0[2];
    volatile uint16_t CR2;
    uint8_t unused1[2];
    volatile uint16_t SMCR;
    uint8_t unused2[2];
    volatile uint16_t DIER;
    uint8_t unused3[2];
    volatile uint16_t SR;
    uint8_t unused4[2];
    volatile uint16_t EGR;
    uint8_t unused5[2];
    volatile uint16_t CCMR1;
    uint8_t unused6[2];
    volatile uint16_t CCMR2;
    uint8_t unused7[2];
    volatile uint16_t CCER;
    uint8_t unused8[2];
    volatile uint32_t CNT;
    volatile uint16_t PSC;
    uint8_t unused9[2];
    volatile uint32_t ARR;
    volatile uint16_t RCR;
    uint8_t unused10[2];
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint16_t BDTR;
    uint8_t unused11[2];
    volatile uint16_t DCR;
    uint8_t unused12[2];
    volatile uint16_t DMAR;
    uint8_t unused13[2];
    volatile uint16_t OR;
    uint8_t unused14[2];
};

/* RTC bit defintions */
#define TIM_EGR_UG          0x0001


#ifdef DEBUG_UART_UNITNO
static struct usart mDbgUart;
#endif
static uint64_t mTimeAccumulated = 0;
static uint32_t mMaxJitterPpm = 0, mMaxDriftPpm = 0, mMaxErrTotalPpm = 0;
static uint32_t mSleepDevsToKeepAlive = 0;
static uint64_t mWakeupTime = 0;
static uint32_t mDevsMaxWakeTime[PLAT_MAX_SLEEP_DEVS] = {0,};
static struct Gpio *mShWakeupGpio;
static struct ChainedIsr mShWakeupIsr;


void platUninitialize(void)
{
#ifdef DEBUG_UART_UNITNO
    usartClose(&mDbgUart);
#endif
}

struct LogBuffer
{
    uint8_t offset;
    char data[255];
} __attribute__((packed));

void *platLogAllocUserData()
{
#if defined(DEBUG_LOG_EVT)
    struct LogBuffer *userData;

    userData = heapAlloc(sizeof(struct LogBuffer));
    if (userData)
        userData->offset = 0;

    return userData;
#else
    return NULL;
#endif
}

void platLogFlush(void *userData)
{
#if defined(DEBUG_LOG_EVT)
    if (userData) {
        if (!osEnqueueEvt(EVENT_TYPE_BIT_DISCARDABLE | DEBUG_LOG_EVT, userData, heapFree, true))
            heapFree(userData);
        else
            hostIntfSetInterrupt(NANOHUB_INT_NONWAKEUP);
    }
#endif
}

bool platLogPutcharF(void *userData, char ch)
{
#if defined(DEBUG) && defined(DEBUG_UART_PIN)
    if (ch == '\n')
        gpioBitbangedUartOut('\r');
    gpioBitbangedUartOut(ch);
#endif
#ifdef DEBUG_UART_UNITNO
    usartPutchat(&mDbgUart, ch);
#elif defined(DEBUG_LOG_EVT)
    struct LogBuffer *buffer;

    if (userData) {
        buffer = userData;
        buffer->data[buffer->offset++] = ch;
    }
#endif
    return true;
}

static bool platWakeupIsr(struct ChainedIsr *isr)
{
    if (!extiIsPendingGpio(mShWakeupGpio))
        return false;

    extiClearPendingGpio(mShWakeupGpio);

    if (gpioGet(mShWakeupGpio) == 0)
        hostIntfSetInterrupt(NANOHUB_INT_WAKE_COMPLETE);
    else
        platReleaseDevInSleepMode(Stm32sleepWakeup);

    return true;
}

void platInitialize(void)
{
    const uint32_t debugStateInSleepMode = 0x00000007; /* debug in all modes */
    struct StmTim *tim = (struct StmTim*)TIM2_BASE;
    struct StmDbg *dbg = (struct StmDbg*)DBG_BASE;
    uint32_t i;

    pwrSystemInit();

    //prepare for sleep mode(s)
    SCB->SCR &=~ SCB_SCR_SLEEPONEXIT_Msk;

    //set ints up for a sane state
    for (i = 0; i < NUM_INTERRUPTS; i++) {
        NVIC_SetPriority(i, 2);
        NVIC_DisableIRQ(i);
        NVIC_ClearPendingIRQ(i);
    }

#ifdef DEBUG_UART_UNITNO
    /* Open mDbgUart on PA2 and PA3 */
    usartOpen(&mDbgUart, DEBUG_UART_UNITNO, DEBUG_UART_GPIO_TX, DEBUG_UART_GPIO_RX,
               115200, USART_DATA_BITS_8,
               USART_STOP_BITS_1_0, USART_PARITY_NONE,
               USART_FLOW_CONTROL_NONE);
#endif

    /* set up debugging */
#if defined(DEBUG) && defined(DEBUG_SWD)
    dbg->CR |= debugStateInSleepMode;
#else
    dbg->CR &=~ debugStateInSleepMode;
#endif

    /* enable MPU */
    mpuStart();

    /* set up timer used for alarms */
    pwrUnitClock(PERIPH_BUS_APB1, PERIPH_APB1_TIM2, true);
    tim->CR1 = (tim->CR1 &~ 0x03E1) | 0x0010; //count down mode with no clock division, disabled
    tim->PSC = 15; // prescale by 16, so that at 16MHz CPU clock, we get 1MHz timer
    tim->DIER |= 1; // interrupt when updated (underflowed)
    tim->ARR = 0xffffffff;
    tim->EGR = TIM_EGR_UG; // force a reload of the prescaler
    NVIC_EnableIRQ(TIM2_IRQn);

    /* set up RTC */
    rtcInit();

    /* bring up systick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    mShWakeupGpio = gpioRequest(SH_INT_WAKEUP);
    gpioConfigInput(mShWakeupGpio, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    syscfgSetExtiPort(mShWakeupGpio);
    extiEnableIntGpio(mShWakeupGpio, EXTI_TRIGGER_BOTH);
    mShWakeupIsr.func = platWakeupIsr;
    extiChainIsr(SH_EXTI_WAKEUP_IRQ, &mShWakeupIsr);
}

static uint64_t platsystickTicksToNs(uint32_t systickTicks)
{
    return (uint64_t)systickTicks * 125 / 2;
}

uint64_t platGetTicks(void)
{
    uint64_t ret;
    uint32_t val;

    do {
        ret = mTimeAccumulated;
        val = SysTick->VAL;
    } while (mTimeAccumulated != ret || SysTick->VAL > val);

    return platsystickTicksToNs(0x01000000 - val) + ret;
}

/* Timer interrupt handler */
void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{
    struct StmTim *tim = (struct StmTim*)TIM2_BASE;

    /* int clear */
    tim->SR &=~ 1;

    /* timer off */
    tim->CR1 &=~ 1;

    /* call timer handler since it might need to reschedule an interrupt (eg: in case where initial delay was too far off & we were limited by timer length) */
    timIntHandler();
}

/* SysTick interrupt handler */
void SysTick_Handler(void);
void SysTick_Handler(void)
{
    mTimeAccumulated += platsystickTicksToNs(SysTick->LOAD + 1); //todo - incremenet by actual elapsed nanoseconds and not just "1"
}

bool platRequestDevInSleepMode(uint32_t sleepDevID, uint32_t maxWakeupTime)
{
    if (sleepDevID >= PLAT_MAX_SLEEP_DEVS || sleepDevID >= Stm32sleepDevNum)
        return false;

    mDevsMaxWakeTime[sleepDevID] = maxWakeupTime;
    while (!atomicCmpXchg32bits(&mSleepDevsToKeepAlive, mSleepDevsToKeepAlive, mSleepDevsToKeepAlive | (1UL << sleepDevID)));

    return true;
}

bool platReleaseDevInSleepMode(uint32_t sleepDevID)
{
    if (sleepDevID >= PLAT_MAX_SLEEP_DEVS || sleepDevID >= Stm32sleepDevNum)
        return false;

    while (!atomicCmpXchg32bits(&mSleepDevsToKeepAlive, mSleepDevsToKeepAlive, mSleepDevsToKeepAlive &~ (1UL << sleepDevID)));

    return true;
}

static uint64_t platSetTimerAlarm(uint64_t delay) //delay at most that many nsec
{
    struct StmTim *tim = (struct StmTim*)TIM2_BASE;

    //turn off timer to prevent interrupts now
    tim->CR1 &=~ 1;

    delay /= 1000;   //to microsecs
    if (delay >> 32) //it is only a 32-bit counter
        delay = 0xffffffff;

    tim->CNT = delay;
    tim->SR &=~ 1; //clear int
    tim->CR1 |= 1;

    return delay;
}

bool platSleepClockRequest(uint64_t wakeupTime, uint32_t maxJitterPpm, uint32_t maxDriftPpm, uint32_t maxErrTotalPpm)
{
    uint64_t intState, curTime = timGetTime();

    if (wakeupTime && curTime >= wakeupTime)
        return false;

    intState = cpuIntsOff();

    mMaxJitterPpm = maxJitterPpm;
    mMaxDriftPpm = maxDriftPpm;
    mMaxErrTotalPpm = maxErrTotalPpm;
    mWakeupTime = wakeupTime;

    //TODO: set an actual alarm here so that if we keep running and do not sleep till this is due, we still fire an interrupt for it!
    if (wakeupTime)
        platSetTimerAlarm(wakeupTime - curTime);

    cpuIntsRestore(intState);

    return true;
}

static bool sleepClockRtcPrepare(uint64_t delay, uint32_t acceptableJitter, uint32_t acceptableDrift, uint32_t maxAcceptableError, void *userData, uint64_t *savedData)
{
    pwrSetSleepType((uint32_t)userData);
    *savedData = rtcGetTime();

    if (delay && rtcSetWakeupTimer(delay) < 0)
        return false;

    //sleep with systick off (for timing) and interrupts off (for power due to HWR errata)
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    return true;
}

static void sleepClockRtcWake(void *userData, uint64_t *savedData)
{
    //re-enable Systic and its interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    mTimeAccumulated += rtcGetTime() - *savedData;
}


static bool sleepClockTmrPrepare(uint64_t delay, uint32_t acceptableJitter, uint32_t acceptableDrift, uint32_t maxAcceptableError, void *userData, uint64_t *savedData)
{
    pwrSetSleepType(stm32f411SleepModeSleep);
    platRequestDevInSleepMode(Stm32sleepDevTim2, 0);

    *savedData = platSetTimerAlarm(delay ?: ~0ull);

    //sleep with systick off (for timing) and interrupts off (for power due to HWR errata)
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
    return true;
}

static void sleepClockTmrWake(void *userData, uint64_t *savedData)
{
    struct StmTim *tim = (struct StmTim*)TIM2_BASE;
    uint64_t leftTicks;

    //re-enable Systic and its interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    //stop the timer counting;
    tim->CR1 &=~ 1;

    leftTicks = tim->CNT; //if we wake NOT from timer, only count the ticks that actually ticked as "time passed"
    if (tim->SR & 1) //if there was an overflow, account for it
        leftTicks -= 0x100000000ull;

    mTimeAccumulated += (*savedData - leftTicks) * 1000; //this clock runs at 1MHz

    platReleaseDevInSleepMode(Stm32sleepDevTim2);
}


static bool sleepClockJustWfiPrepare(uint64_t delay, uint32_t acceptableJitter, uint32_t acceptableDrift, uint32_t maxAcceptableError, void *userData, uint64_t *savedData)
{
    pwrSetSleepType(stm32f411SleepModeSleep);

    return true;
}

struct PlatSleepAndClockInfo {
    uint64_t resolution;
    uint32_t maxCounter;
    uint32_t jitterPpm;
    uint32_t driftPpm;
    uint32_t maxWakeupTime;
    uint32_t devsAvail; //what is available in sleep mode?
    bool (*prepare)(uint64_t delay, uint32_t acceptableJitter, uint32_t acceptableDrift, uint32_t maxAcceptableError, void *userData, uint64_t *savedData);
    void (*wake)(void *userData, uint64_t *savedData);
    void *userData;
} static const platSleepClocks[] = {

    { /* RTC + LPLV STOP MODE */
        .resolution = 1000000000ull/32768,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 50,
        .maxWakeupTime = 407000ull,
        .prepare = sleepClockRtcPrepare,
        .wake = sleepClockRtcWake,
        .userData = (void*)stm32f411SleepModeStopLPLV,
    },
    { /* RTC + LPFD STOP MODE */
        .resolution = 1000000000ull/32768,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 50,
        .maxWakeupTime = 130000ull,
        .prepare = sleepClockRtcPrepare,
        .wake = sleepClockRtcWake,
        .userData = (void*)stm32f411SleepModeStopLPFD,
    },
    { /* RTC + MRFPD STOP MODE */
        .resolution = 1000000000ull/32768,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 50,
        .maxWakeupTime = 111000ull,
        .prepare = sleepClockRtcPrepare,
        .wake = sleepClockRtcWake,
        .userData = (void*)stm32f144SleepModeStopMRFPD,
    },
    { /* RTC + MR STOP MODE */
        .resolution = 1000000000ull/32768,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 50,
        .maxWakeupTime = 14500ull,
        .prepare = sleepClockRtcPrepare,
        .wake = sleepClockRtcWake,
        .userData = (void*)stm32f144SleepModeStopMR,
    },
    { /* TIM2 + SLEEP MODE */
        .resolution = 1000000000ull/1000000,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 30,
        .maxWakeupTime = 12ull,
        .devsAvail = (1 << Stm32sleepDevTim2) | (1 << Stm32sleepDevTim4) | (1 << Stm32sleepDevTim5) | (1 << Stm32sleepWakeup) | (1 << Stm32sleepDevSpi2) | (1 << Stm32sleepDevI2c1),
        .prepare = sleepClockTmrPrepare,
        .wake = sleepClockTmrWake,
    },
    { /* just WFI */
        .resolution = 16000000000ull/1000000,
        .maxCounter = 0xffffffff,
        .jitterPpm = 0,
        .driftPpm = 0,
        .maxWakeupTime = 0,
        .devsAvail = (1 << Stm32sleepDevTim2) | (1 << Stm32sleepDevTim4) | (1 << Stm32sleepDevTim5) | (1 << Stm32sleepWakeup) | (1 << Stm32sleepDevSpi2) | (1 << Stm32sleepDevI2c1),
        .prepare = sleepClockJustWfiPrepare,
    },

    /* terminator */
    {0},
};

void platSleep(void)
{
    uint64_t predecrement = 0, curTime = timGetTime(), length = mWakeupTime - curTime, intState;
    const struct PlatSleepAndClockInfo *sleepClock, *leastBadOption = NULL;
    uint64_t savedData;
    uint32_t i;

    //shortcut the sleep if it is time to wake up already
    if (mWakeupTime && mWakeupTime < curTime)
        return;

    for (sleepClock = platSleepClocks; sleepClock->maxCounter; sleepClock++) {

        bool potentialLeastBadOption = false;

        //if we have timers, consider them
        if (mWakeupTime) {

            //calculate how much we WOULD predecerement by
            predecrement = sleepClock->resolution + sleepClock->maxWakeupTime;

            //skip options with too much jitter (after accounting for error
            if (sleepClock->jitterPpm > mMaxJitterPpm)
                continue;

            //skip options that will take too long to wake up to be of use
            if (predecrement > length)
                continue;

            //skip options with too much  drift
            if (sleepClock->driftPpm > mMaxDriftPpm)
                continue;

            //skip options that do not let us sleep enough, but save them for later if we simply must pick something
            if (length / sleepClock->resolution > sleepClock->maxCounter && !leastBadOption)
                potentialLeastBadOption = true;
        }

        //skip all options that do not keep enough deviceas awake
        if ((sleepClock->devsAvail & mSleepDevsToKeepAlive) != mSleepDevsToKeepAlive)
            continue;

        //skip all options that wake up too slowly
        for (i = 0; i < Stm32sleepDevNum; i++) {
            if (!(mSleepDevsToKeepAlive & (1 << i)))
                continue;
            if (mDevsMaxWakeTime[i] < sleepClock->maxWakeupTime)
                break;
        }
        if (i != Stm32sleepDevNum)
            continue;

        //if it will not let us sleep long enough save it as a possibility and go on
        if (potentialLeastBadOption && !leastBadOption)
            leastBadOption = sleepClock;
        else //if it fits us perfectly, pick it
            break;
    }
    if (!sleepClock->maxCounter)
        sleepClock = leastBadOption;

    if (!sleepClock) {
        //should never happen - this will spin the CPU and be bad, but it WILL work in all cases
        return;
    }

    //turn ints off in prep for sleep
    intState = cpuIntsOff();

    //options? config it
    if (sleepClock->prepare && !sleepClock->prepare(mWakeupTime ? length - sleepClock->maxWakeupTime : 0, mMaxJitterPpm, mMaxDriftPpm, mMaxErrTotalPpm, sleepClock->userData, &savedData))
        return;

    asm volatile ("wfi\n"
        "nop" :::"memory");

    //wakeup
    if (sleepClock->wake)
        sleepClock->wake(sleepClock->userData, &savedData);

    //re-enable interrupts and let the handlers run
    cpuIntsRestore(intState);
}
