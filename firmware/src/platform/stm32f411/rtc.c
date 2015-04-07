#include <plat/inc/rtc.h>
#include <plat/inc/pwr.h>
#include <inc/timer.h>
#include <inc/platform.h>
#include <inc/exti.h>
#include <plat/inc/cmsis.h>

#ifndef NS_PER_S
#define NS_PER_S                    1000000000UL
#endif

struct StmRtc
{
    volatile uint32_t TR;
    volatile uint32_t DR;
    volatile uint32_t CR;
    volatile uint32_t ISR;
    volatile uint32_t PRER;
    volatile uint32_t WUTR;
    volatile uint32_t CALIBR;
    volatile uint32_t ALRMAR;
    volatile uint32_t ALRMBR;
    volatile uint32_t WPR;
    volatile uint32_t SSR;
    volatile uint32_t TSTR;
    volatile uint32_t TSSSR;
    volatile uint32_t CALR;
    volatile uint32_t TAFCR;
    volatile uint32_t ALRMASSR;
    volatile uint32_t ALRMBSSR;
    uint8_t unused0[4];
    volatile uint32_t BKPR[20];
};

#define RTC ((struct StmRtc*)RTC_BASE)

/* RTC bit defintions */
#define RTC_CR_WUCKSEL_MASK         0x00000007UL
#define RTC_CR_WUCKSEL_16DIV        0x00000000UL
#define RTC_CR_WUCKSEL_8DIV         0x00000001UL
#define RTC_CR_WUCKSEL_4DIV         0x00000002UL
#define RTC_CR_WUCKSEL_2DIV         0x00000003UL
#define RTC_CR_FMT                  0x00000040UL
#define RTC_CR_ALRAE                0x00000100UL
#define RTC_CR_WUTE                 0x00000400UL
#define RTC_CR_ALRAIE               0x00001000UL
#define RTC_CR_WUTIE                0x00004000UL

#define RTC_ISR_ALRAWF              0x00000001UL
#define RTC_ISR_WUTWF               0x00000004UL
#define RTC_ISR_RSF                 0x00000020UL
#define RTC_ISR_INITF               0x00000040UL
#define RTC_ISR_INIT                0x00000080UL
#define RTC_ISR_WUTF                0x00000400UL

/* RTC internal values */
#define RTC_FREQ_HZ                 32768UL
#define RTC_WKUP_DOWNCOUNT_MAX      0x10000UL

/* TODO: Reset to crystal PPM once known */
#define RTC_PPM                     50UL

/* Default prescalars of P[async] = 127 and P[sync] = 255 are appropriate
 * produce a 1 Hz clock when using a 32.768kHZ clock source */
#define RTC_PREDIV_A                127UL
#define RTC_PREDIV_S                255UL

/* Jitter = max wakeup timer resolution (61.035 us)
 * + 2 RTC cycles for synchronization (61.035 us) */
#define RTC_PERIOD_NS               30517UL
#define RTC_CK_APRE_HZ              256UL
#define RTC_CK_APRE_PERIOD_NS       3906250UL
#define RTC_DIV2_PERIOD_NS          61035UL
#define RTC_DIV4_PERIOD_NS          122070UL
#define RTC_DIV8_PERIOD_NS          244141UL
#define RTC_DIV16_PERIOD_NS         488281UL
/* TODO: Measure the jitter in the overhead of setting the wakeup timers.
 * Initially setting to 1 RTC clock cycle. */
#define RTC_WUT_NOISE_NS            30517UL
#define RTC_ALARM_NOISE_NS          30517UL

static void rtcSetDefaultDateTimeAndPrescalar(void)
{
    /* Enable writability of RTC registers */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter RTC init mode */
    RTC->ISR |= RTC_ISR_INIT;

    asm volatile("":::"memory");
    /* Wait for initialization mode to be entered. */
    while ((RTC->ISR & RTC_ISR_INITF) == 0);

    /* Set prescalar rtc register.  Two writes required. */
    RTC->PRER |= (RTC_PREDIV_A << 16);
    RTC->PRER |= RTC_PREDIV_S;

    /* 24 hour format */
    RTC->CR &= ~RTC_CR_FMT;
    /* Set time and date registers to defaults */
    /* Midnight */
    RTC->TR = 0x0;
    RTC->SSR = 0x0;
    /* Sat Jan 1st, 2000 BCD */
    RTC->DR = 0b1100000100000001;

    /* Exit init mode for RTC */
    RTC->ISR &= ~RTC_ISR_INIT;
    /* Re-enable register write protection.  RTC counting doesn't start for
     * 4 RTC cycles after set - must poll RSF before read DR or TR */
    RTC->WPR = 0xFF;

    extiEnableIntLine(EXTI_LINE_RTC_WKUP, EXTI_TRIGGER_RISING);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

void rtcInit(void)
{
    pwrEnableAndClockRtc();
    rtcSetDefaultDateTimeAndPrescalar();
}

/* Set calendar alarm to go off after delay has expired. nanotime_t delay must
 * be in valid nanotime_t format and must be less than 32 s.  A negative value
 * for the 'ppm' param indicates the alarm has no accuracy requirements. */
int rtcSetWakeupTimer(struct nanotime_t delay, int ppm)
{
    if (delay.time_s >= 32) {
        return TIMER_ERR_TOO_BIG;
    }

    uint64_t delayNs = delay.time_ns + delay.time_s * (uint64_t)NS_PER_S;
    uint32_t wakeupClock;
    uint32_t periodNs;

    /* Minimum wakeup interrupt period is 122 us */
    if (delayNs < (RTC_DIV2_PERIOD_NS * 2)) {
        return TIMER_ERR_TOO_SMALL;
    }

    /* Get appropriate clock period for delay size.  Wakeup clock = RTC/x. */
    if (delayNs < (RTC_DIV2_PERIOD_NS * RTC_WKUP_DOWNCOUNT_MAX)) {
        wakeupClock = RTC_CR_WUCKSEL_2DIV;
        periodNs = RTC_DIV2_PERIOD_NS;
    } else if (delayNs < ((unsigned long long)RTC_DIV4_PERIOD_NS *
                RTC_WKUP_DOWNCOUNT_MAX)) {
        wakeupClock = RTC_CR_WUCKSEL_4DIV;
        periodNs = RTC_DIV4_PERIOD_NS;
    } else if (delayNs < ((unsigned long long)RTC_DIV8_PERIOD_NS *
                RTC_WKUP_DOWNCOUNT_MAX)) {
        wakeupClock = RTC_CR_WUCKSEL_8DIV;
        periodNs = RTC_DIV8_PERIOD_NS;
    } else if (delayNs < ((unsigned long long)RTC_DIV16_PERIOD_NS *
                RTC_WKUP_DOWNCOUNT_MAX)) {
        wakeupClock = RTC_CR_WUCKSEL_16DIV;
        periodNs = RTC_DIV16_PERIOD_NS;
    } else {
        osLog(LOG_ERROR, "Y U NO RETURN ERROR?");
        return TIMER_ERR_EVERYTHING_IS_TERRIBLE;
    }

    /* If PPM for OS timer is less than PPM of wakeup timer, can't use.
     * PPM of timer is calculated as jitter/(delay * 1,000,000) + drift PPM */
    if ((ppm - RTC_PPM) * delayNs < (periodNs + RTC_WUT_NOISE_NS)* 1000000)
        return TIMER_ERR_ACCURACY_REQUIREMENTS_UNMET;

    platDisableInterrupts();

    pwrEnableWriteBackupDomainRegs();

    /* Enable RTC register write */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter init mode */
    RTC->ISR |= RTC_ISR_INIT;

    while ((RTC->ISR & RTC_ISR_INITF) == 0);

    /* Disable wakeup timer */
    RTC->CR &= ~RTC_CR_WUTE;

    /* Wait for access enabled for wakeup timer registers */
    while ((RTC->ISR & RTC_ISR_WUTWF) == 0);

    /* Clear wakeup clock source*/
    RTC->CR &= ~RTC_CR_WUCKSEL_MASK;

    RTC->CR |= wakeupClock;
    /* Downcounter value for wakeup clock.  Wakeup flag is set every
     * RTC->WUTR[15:0] + 1 cycles of the WUT clock. */
    RTC->WUTR = delayNs / periodNs - 1;

    /* Enable wakeup interrupts */
    RTC->CR |= RTC_CR_WUTIE;
    extiClearPendingLine(EXTI_LINE_RTC_WKUP);
    /* Enable wakeup timer */
    RTC->CR |= RTC_CR_WUTE;
    /* Exit init mode */
    RTC->ISR &= ~RTC_ISR_INIT;
    /* Clear overflow flag */
    RTC->ISR &=~RTC_ISR_WUTF;
    /* Write-protect RTC registers */
    RTC->WPR = 0xFF;

    platEnableInterrupts();

    return 0;
}

void EXTI22_RTC_WKUP_IRQHandler(void);
void EXTI22_RTC_WKUP_IRQHandler(void)
{
    extiClearPendingLine(EXTI_LINE_RTC_WKUP);
}
