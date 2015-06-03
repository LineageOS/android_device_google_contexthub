#include <string.h>

#include <crc.h>
#include <seos.h>

#include <plat/inc/pwr.h>

struct StmCrcRegs {
    volatile uint32_t DR;
    volatile uint32_t IDR;
    volatile uint32_t CR;
};

#define STM_CRC_CR_RESET        1

static struct StmCrcRegs *gCrcRegs = (struct StmCrcRegs *)CRC_BASE;

uint32_t crc32(const void *buf, size_t size)
{
    const uint32_t *words = (const uint32_t *)buf;
    size_t numWords = size / 4;
    unsigned int leftoverBytes = size % 4;

    pwrUnitClock(PERIPH_BUS_AHB1, PERIPH_AHB1_CRC, true);

    gCrcRegs->CR = STM_CRC_CR_RESET;
    size_t i;
    for (i = 0; i < numWords; i++)
        gCrcRegs->DR = words[i];

    if (leftoverBytes) {
        uint32_t word = 0;
        memcpy(&word, words + numWords, leftoverBytes);
        /* n.b.: no shifting is needed, since the CRC block looks at the
         * lowest byte first (i.e., we need the padding in the upper bytes)
         */
        gCrcRegs->DR = word;
    }

    uint32_t crc = gCrcRegs->DR;
    pwrUnitClock(PERIPH_BUS_AHB1, PERIPH_AHB1_CRC, false);
    return crc;
}
