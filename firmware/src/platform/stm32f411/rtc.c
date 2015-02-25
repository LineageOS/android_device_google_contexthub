#include <plat/inc/rtc.h>

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
    volatile uint32_t SHIFTR;
    volatile uint32_t TSTR;
    volatile uint32_t TSDR;
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
#define RTC_CR_FMT 0x00000001UL

bool rtcInit(void)
{
    pwrSetUpRtc();

    /* Enable writability of RTC registers */
    TC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter RTC init mode */
    RTC->ISR = 0x80;

    /* Wait for initialization mode to be entered. */
    while ((RTC->ISR & RTC_ISR_INITF) == 0);

    /* Default prescalars of P[async] = 127 and P[sync] = 255 are appropriate
     * produce a 1 Hz clock when using a 32.768kHZ clock source */
    uint32_t async_prescalar = 127;
    uint32_t sync_prescalar = 255;

    /* Set prescalar rtc regiseter */
    RTC->PRER |= (async_prescalar << 16) + sync_prescalar;

    /* 24 hour format */
    RTC->CR &= ~RTC_CR_FMT;
    /* Set time and date registers arbitrarily */
    RTC->TR = 0x000000;
    RTC->DR = 0x146712;

    /* Exit init mode for RTC */
    RTC->ISR = 0x0;
    RTC->WPR = 0xFF;

    pwrDisableBackupDomainWriteAccess();
    /* Disable write permissions for backup domain */
    PWR->CR &= ~PWR_CR_DBP;
}
