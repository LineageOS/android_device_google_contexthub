#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#ifdef __cplusplus
}
#endif


#include <stdint.h>

#define BL_VERSION_1        1
#define BL_VERSION_CUR        BL_VERSION_1

struct BlVecTable {
    /* cortex requirements */
    uint32_t    bl_stack_top;
    void        (*bl_entry)(void);
    void        (*bl_nmi_handler)(void);
    void        (*bl_hard_fault_handler)(void);
    void        (*bl_mmu_fault_handler)(void);
    void        (*bl_bus_fault_handler)(void);
    void        (*bl_usage_fault_handler)(void);
    
    /* bl api */
    //ver 1 bl supports:
    uint32_t    (*bl_get_version)(void);
    void        (*bl_reboot)(void);
};



//for using outside of bootloader
extern char __bl_start[];
#define BL    ((struct BlVecTable*)&__bl_start)


#ifdef __cplusplus
}
#endif

#endif

