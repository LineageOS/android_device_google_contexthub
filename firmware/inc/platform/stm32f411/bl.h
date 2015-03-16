#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#ifdef __cplusplus
}
#endif

#include <stdint.h>

#define BL_VERSION_1        1
#define BL_VERSION_CUR      BL_VERSION_1

#define BL_FLASH_KERNEL_ID  0x1
#define BL_FLASH_EEDATA_ID  0x2
#define BL_FLASH_APP_ID     0x4

#define BL_FLASH_KEY1       0x45670123
#define BL_FLASH_KEY2       0xCDEF89AB

struct BlVecTable {
    /* cortex requirements */
    uint32_t    blStackTop;
    void        (*blEntry)(void);
    void        (*blNmiHandler)(void);
    void        (*blHardFaultHandler)(void);
    void        (*blMmuFaultHandler)(void);
    void        (*blBusFaultHandler)(void);
    void        (*blUsageFaultHandler)(void);

    /* bl api */
    //ver 1 bl supports:
    uint32_t    (*blGetVersion)(void);
    void        (*blReboot)(void);
    void        (*blGetSnum)(uint32_t *snum, uint8_t length);
    int         (*blProgramShared)(uint8_t *dst, uint8_t *src, uint32_t length, uint32_t key1, uint32_t key2);
};

//for using outside of bootloader
extern char __bl_start[];
#define BL    ((struct BlVecTable*)&__bl_start)

#ifdef __cplusplus
}
#endif

#endif
