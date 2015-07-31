#include <plat/inc/rtc.h>
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
#include <seos.h>
#include <mpu.h>
#include <cpu.h>


void platUninitialize(void)
{
    //TODO
}

void platSleep(void)
{
    //TODO
}

void platWake(void)
{
    //TODO
}

void platLogPutchar(char ch)
{
     putchar(ch);
}

void platInitialize(void)
{
    /* set up RTC */
    rtcInit();

    //TODO
}

uint64_t platGetTicks(void)
{
    //TODO

    return 0;
}

int main(int argc, char** argv)
{
    osMain();

    return 0;
}
