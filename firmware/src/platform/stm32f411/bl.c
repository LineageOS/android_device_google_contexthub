#include <plat/inc/cmsis.h>
#include <plat/inc/pwr.h>
#include <plat/inc/bl.h>

#define BOOTLOADER    __attribute__ ((section (".bltext")))

//linker provides these
extern char __stack_top[];
extern char __bl_start[];
extern void __VECTORS();



void BOOTLOADER __bl_entry(void);
void BOOTLOADER __bl_entry(void)
{
    uint32_t appBase = ((uint32_t)&__VECTORS) &~ 1;

    //make sure we're the vector table and no ints happen (BL does not use them)
    asm volatile("cpsid i");
    SCB->VTOR = (uint32_t)&__bl_start;


    //todo


    //call main app with ints off
    asm volatile("cpsid i");
    SCB->VTOR = appBase;
    asm volatile(
        "LDR SP, [%0, #0]    \n"
        "LDR PC, [%0, #4]    \n"
        :
        :"r"(appBase)
        :"memory", "cc"
    );

    //we should never return here
    while(1);
}

static uint32_t BOOTLOADER __bl_version(void)
{
    return BL_VERSION_CUR;
}

static void BOOTLOADER __bl_reboot(void)
{
    SCB->AIRCR = 0x05FA0004;
    //we never get here
    while(1);
}

static void __bl_spurious(void)
{
    //BAD!
    __bl_reboot();
}

const struct BlVecTable __attribute__((section(".blvec"))) __BL_VECTORS =
{
    /* cortex */
    .bl_stack_top = (uint32_t)&__stack_top,
    .bl_entry = &__bl_entry,
    .bl_nmi_handler = &__bl_spurious,
    .bl_mmu_fault_handler = &__bl_spurious,
    .bl_bus_fault_handler = &__bl_spurious,
    .bl_usage_fault_handler = &__bl_spurious,

    /* api */
    .bl_get_version = &__bl_version,
    .bl_reboot = &__bl_reboot,
};
