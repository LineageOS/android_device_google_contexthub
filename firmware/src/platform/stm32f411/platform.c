#include <plat/inc/gpio.h>
#include <plat/inc/usart.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/pwr.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <platform.h>
#include <seos.h>
#include <timer.h>
#include <usart.h>
#include <gpio.h>


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


/* Use mUsart2 for kernel logging. */
static struct usart mUsart2;
static uint64_t mTicks = 0;

static void platInitializeDebug();
static void platInitializeTimer(void);


void platInitialize(void)
{
    uint32_t i;

    pwrSystemInit();

    //set ints up for a sane state
    for (i = 0; i < NUM_INTERRUPTS; i++) {
        NVIC_SetPriority(i, 1);
        NVIC_DisableIRQ(i);
        NVIC_ClearPendingIRQ(i);
    }

    /* Open mUsart2 on PA2 and PA3 */
    usart_open(&mUsart2, 2, GPIO_PA(2), GPIO_PA(3),
               115200, USART_DATA_BITS_8,
               USART_STOP_BITS_1_0, USART_PARITY_NONE,
               USART_FLOW_CONTROL_NONE);

    platInitializeDebug();
    platInitializeTimer();
}

void platUninitialize(void)
{
    usart_close(&mUsart2);
}

void platSleep(void)
{
    asm volatile ("wfi" :::"memory");
}

void platWake(void)
{
    osLog(LOG_ERROR, "Wake unimplemented.");
}

void platLog(char *string)
{
    while (*string != '\0')
        usart_putchar(&mUsart2, *string++);
    usart_putchar(&mUsart2, '\n');
}

void platDisableInterrupts(void)
{
    asm volatile("cpsid i");
}

void platEnableInterrupts(void)
{
    asm volatile("cpsie i");
}

static void platInitializeDebug()
{
    struct StmDbg *dbg = (struct StmDbg*)DBG_BASE;
    const uint32_t debugStateInSleepMode = 0x00000001;

#ifdef DEBUG
    dbg->CR |= debugStateInSleepMode;
#else
    dbg->CR &=~ debugStateInSleepMode;
#endif
}

static void platInitializeTimer()
{
    struct StmTim *block = (struct StmTim*)TIM2_BASE;

    /* Enable clock for timer */
    pwrUnitClock(PERIPH_BUS_APB1, PERIPH_APB1_TIM2, true);

    /* count down mode with no clock division, disabled */
    block->CR1 = (block->CR1 &~ 0x03E1) | 0x0010;
    block->PSC = 15; /* prescale by 16, so that at 16MHz CPU clock, we get 1MHz timer */
    /* block->ARR is where we'd stash the number of microseconds we want to count down from */
    block->DIER |= 1; /* interrupt when updated (underflowed) */

    /* int on*/
    NVIC_EnableIRQ(TIM2_IRQn);
}

///* Provides a simple console to SEOS */
//static void *platConsoleThreadFunc(void *arg)
//{
//    char buffer[40];
//    bool running = true;
//
//    while (running) {
//        printf("$ ");
//
//        /* Read and terminate line from stdin */
//        fgets(buffer, sizeof(buffer), stdin);
//        buffer[strlen(buffer) - 1] = '\0';
//
//        if (strcmp(buffer, "exit") == 0)
//            running = false;
//        else if (strcmp(buffer, "halt") == 0) {
//            osSystemCall(SYSTEMCALLHALT, NULL, 0);
//
//            /* Simulate a CPU wake */
//            platWake();
//        }
//    }
//
//    printf("Console disconnected\n");
//
//    return NULL;
//}
//
///* Provides some dummy sensor data */
//static void *platSensorThreadFunc(void *arg)
//{
//    while (true) {
//        /*
//         * Without file descriptors or files, just use
//         * a syscall to push sensor data over to SEOS
//         */
//        osSystemCall(SYSTEMCALLSENSOR, NULL, 0);
//
//        /* Simulate a CPU wake */
//        platWake();
//
//        sleep(10);
//    }
//
//    return NULL;
//}

/* RTC/alarm */
unsigned platGetRtcMs(void)
{
    osLog(LOG_ERROR, "Unimplemented.");
    return 0;
}

void platSetAlarm(unsigned delayUs)
{
    struct StmTim *block = (struct StmTim*)TIM2_BASE;

    /* XXX: assure no alarm already pending in here */

    block->CNT = delayUs;
    block->CR1 |= 1;
}

unsigned platGetSystick(void)
{
    return mTicks;
}

/* Timer interrupt handler */
void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{
    struct StmTim *block = (struct StmTim*)TIM2_BASE;

    /* int clear */
    block->SR &=~ 1;

    /* timer off */
    block->CR1 &=~ 1;

    /* tell the caller */
    Timer_interrupt_handler();
}

/* SysTick interrupt handler */
void SysTick_Handler(void);
void SysTick_Handler(void)
{
    mTicks++;
}

/* TODO: move this to interrupts.c */
void HardFault_Handler(void);
void HardFault_Handler(void)
{
    while (1){}
}

