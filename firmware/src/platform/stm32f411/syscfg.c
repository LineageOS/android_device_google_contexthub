#include <plat/inc/pwr.h>
#include <plat/inc/syscfg.h>

#define SYSCFG_REG_SHIFT 2

struct StmSyscfg
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t CMPCR;
};

void syscfgSetExtiPort(const struct gpio *__restrict gpio)
{
    struct StmSyscfg *block = (struct StmSyscfg *)SYSCFG_BASE;
    const uint32_t bankNo = gpio->gpio >> GPIO_PORT_SHIFT;
    const uint32_t pinNo = gpio->gpio & GPIO_PIN_MASK;
    const uint32_t regNo = pinNo >> SYSCFG_REG_SHIFT;
    const uint32_t nibbleNo = pinNo & ((1UL << SYSCFG_REG_SHIFT) - 1UL);
    const uint32_t shift_4b = nibbleNo << 2UL;
    const uint32_t mask_4b = 0x0FUL << shift_4b;

    pwrUnitClock(PERIPH_BUS_APB2, PERIPH_APB2_SYSCFG, true);

    block->EXTICR[regNo] = (block->EXTICR[regNo] & ~mask_4b) | (bankNo << shift_4b);
}
