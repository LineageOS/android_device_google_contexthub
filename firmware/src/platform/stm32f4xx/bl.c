/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define SECURITY_API_IN_BOOTLOADER
#include <plat/inc/cmsis.h>
#include <plat/inc/pwr.h>
#include <plat/inc/bl.h>
#include <alloca.h>
#include <aes.h>
#include <sha2.h>
#include <rsa.h>


struct StmCrc
{
    volatile uint32_t DR;
    volatile uint32_t IDR;
    volatile uint32_t CR;
};

struct StmFlash
{
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
};

struct StmRcc {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint8_t unused0[4];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint8_t unused1[8];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint8_t unused2[4];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint8_t unused3[8];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint8_t unused4[4];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint8_t unused5[8];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint8_t unused6[8];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
};

struct StmUdid
{
    volatile uint32_t U_ID[3];
};

#define BL_MAX_FLASH_CODE   1024

#define STM32F4_CRC_RESIDUE 0xC704DD7B

#define UDID_BASE           0x1FFF7A10

#define CRC_CR_RESET        0x00000001

#define FLASH_ACR_LAT(x)    ((x) & FLASH_ACR_LAT_MASK)
#define FLASH_ACR_LAT_MASK  0x0F
#define FLASH_ACR_PRFTEN    0x00000100
#define FLASH_ACR_ICEN      0x00000200
#define FLASH_ACR_DCEN      0x00000400
#define FLASH_ACR_ICRST     0x00000800
#define FLASH_ACR_DCRST     0x00001000

#define FLASH_SR_EOP        0x00000001
#define FLASH_SR_OPERR      0x00000002
#define FLASH_SR_WRPERR     0x00000010
#define FLASH_SR_PGAERR     0x00000020
#define FLASH_SR_PGPERR     0x00000040
#define FLASH_SR_PGSERR     0x00000080
#define FLASH_SR_RDERR      0x00000100
#define FLASH_SR_BSY        0x00010000

#define FLASH_CR_PG         0x00000001
#define FLASH_CR_SER        0x00000002
#define FLASH_CR_MER        0x00000004
#define FLASH_CR_SNB(x)     (((x) << FLASH_CR_SNB_SHIFT) & FLASH_CR_SNB_MASK)
#define FLASH_CR_SNB_MASK   0x00000078
#define FLASH_CR_SNB_SHIFT  3
#define FLASH_CR_PSIZE(x)   (((x) << FLASH_CR_PSIZE_SHIFT) & FLASH_CR_PSIZE_MASK)
#define FLASH_CR_PSIZE_MASK 0x00000300
#define FLASH_CR_PSIZE_SHIFT 8
#define FLASH_CR_PSIZE_8    0x0
#define FLASH_CR_PSIZE_16   0x1
#define FLASH_CR_PSIZE_32   0x2
#define FLASH_CR_PSIZE_64   0x3
#define FLASH_CR_STRT       0x00010000
#define FLASH_CR_EOPIE      0x01000000
#define FLASH_CR_ERRIE      0x02000000
#define FLASH_CR_LOCK       0x80000000

typedef void (*flashErase)(volatile uint32_t *, uint32_t, volatile uint32_t *);
typedef void (*flashWrite)(volatile uint8_t *, uint8_t, volatile uint32_t *);

//linker provides these
extern char __stack_top[];
extern char __ram_start[];
extern char __ram_end[];
extern char __eedata_start[];
extern char __eedata_end[];
extern char __code_start[];
extern char __code_end[];
extern char __shared_start[];
extern char __shared_end[];
extern void __VECTORS();

void BOOTLOADER __blEntry(void);
static void BOOTLOADER __blCopyShared(void);
static int BOOTLOADER __blProgramFlash(uint8_t *, uint8_t *, uint32_t, uint32_t, uint32_t);
static uint32_t BOOTLOADER __blCrcCont(uint8_t *, unsigned int);
static uint32_t BOOTLOADER __blCrc(uint8_t *, unsigned int);
static void BOOTLOADER __blEraseSectors(uint32_t, uint8_t *);
static void BOOTLOADER __blWriteBytes(uint8_t *, uint8_t *, uint32_t);
static int BOOTLOADER __blCompareBytes(uint8_t *, uint8_t *, uint32_t);

static inline int BOOTLOADER __blPad(int length)
{
    return (length + 3) & ~3;
}

static inline uint32_t BOOTLOADER __blDisableInts(void)
{
    uint32_t state;

    asm volatile (
        "mrs %0, PRIMASK    \n"
        "cpsid i            \n"
        :"=r"(state)
    );

    return state;
}

static inline void BOOTLOADER __blRestoreInts(uint32_t state)
{
    asm volatile(
        "msr PRIMASK, %0   \n"
        ::"r"((uint32_t)state)
    );
}

void BOOTLOADER __blEntry(void)
{
    uint32_t appBase = ((uint32_t)&__VECTORS) & ~1;

    //make sure we're the vector table and no ints happen (BL does not use them)
    __blDisableInts();
    SCB->VTOR = (uint32_t)&BL;

    __blCopyShared();

    //call main app with ints off
    __blDisableInts();
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

static uint32_t BOOTLOADER __blVersion(void)
{
    return BL_VERSION_CUR;
}

static void BOOTLOADER __blReboot(void)
{
    SCB->AIRCR = 0x05FA0004;
    //we never get here
    while(1);
}

static void BOOTLOADER __blGetSnum(uint32_t *snum, uint8_t length)
{
    struct StmUdid *reg = (struct StmUdid *)UDID_BASE;
    int i;

    if (length > 3)
        length = 3;

    for (i=0; i<length; i++)
        snum[i] = reg->U_ID[i];
}

static void BOOTLOADER __blCopyShared()
{
    struct StmRcc *rcc = (struct StmRcc *)RCC_BASE;
    uint8_t *shared_start = (uint8_t *)&__shared_start;
    uint8_t *shared_end = (uint8_t *)&__shared_end;
    uint8_t *code_start = (uint8_t *)&__code_start;
    uint8_t *code_end = (uint8_t *)&__code_end;
    uint8_t *eedata_start = (uint8_t *)&__eedata_start;
    uint8_t *eedata_end = (uint8_t *)&__eedata_end;
    uint8_t *shared, *data, *dest = 0;
    int len, total_len;
    uint8_t id1, id2;
    uint32_t crc, new_crc, old_crc;

    /* enable crc clock */
    rcc->AHB1ENR |= PERIPH_AHB1_CRC;

    //copy shared to code and eedata
    for (shared = shared_start;
         shared < shared_end && shared[0] != 0xFF;
         shared += total_len) {
        id1 = shared[0] & 0x0F;
        id2 = (shared[0] >> 4) & 0x0F;
        len = (shared[1] << 16) | (shared[2] << 8) | shared[3];
        total_len = sizeof(uint32_t) + __blPad(len) + sizeof(uint32_t);
        data = &shared[4];

        if (shared + total_len > shared_end)
            break;

        //skip over erased sections
        if (id1 != id2)
            continue;

        if (id1 == BL_FLASH_KERNEL_ID || id1 == BL_FLASH_EEDATA_ID) {
            crc = __blCrc(shared, total_len);
            if (crc == STM32F4_CRC_RESIDUE) {
                if (id1 == BL_FLASH_KERNEL_ID) {
                    dest = code_start;
                    if (dest + len > code_end)
                        break;
                } else if (id1 == BL_FLASH_EEDATA_ID) {
                    dest = eedata_start;
                    if (dest + len > eedata_end)
                        break;
                }

                if (__blProgramFlash(dest, data, len, BL_FLASH_KEY1, BL_FLASH_KEY2) == 0) {
                    old_crc = __blCrc(data, len);
                    new_crc = __blCrc(dest, len);

                    if (old_crc == new_crc) {
                        // mark as erased by zero-ing the upper 4 bits
                        uint8_t erased = id1;
                        __blProgramFlash(shared, &erased, 1, BL_FLASH_KEY1, BL_FLASH_KEY2);
                    }
                }
            }
        }
    }

    /* disable crc clock */
    rcc->AHB1ENR &= ~PERIPH_AHB1_CRC;
}

enum blFlashType
{
    BL_FLASH_BL,
    BL_FLASH_EEDATA,
    BL_FLASH_KERNEL,
    BL_FLASH_SHARED
};

// For erase need to know which page a given memory address is in
static const BOOTLOADER_RO struct blFlashTable
{
    uint8_t *address;
    uint32_t length;
    uint32_t type;
} __blFlashTable[] =
{
    { (uint8_t *)(&BL),                      0x04000, BL_FLASH_BL     },
    { (uint8_t *)(__eedata_start),           0x04000, BL_FLASH_EEDATA },
    { (uint8_t *)(__eedata_start + 0x04000), 0x04000, BL_FLASH_EEDATA },
    { (uint8_t *)(__code_start),             0x04000, BL_FLASH_KERNEL },
    { (uint8_t *)(__code_start + 0x04000),   0x10000, BL_FLASH_KERNEL },
    { (uint8_t *)(__shared_start),           0x20000, BL_FLASH_SHARED },
    { (uint8_t *)(__shared_start + 0x20000), 0x20000, BL_FLASH_SHARED },
    { (uint8_t *)(__shared_start + 0x40000), 0x20000, BL_FLASH_SHARED },
};

/*
 * Return the address of the erase code and the length of the code
 *
 * This code needs to run out of ram and not flash since accessing flash
 * while erasing is undefined (best case the processor stalls, worst case
 * it starts executing garbage)
 *
 * This function is used to get a pointer to the actual code that does the
 * erase and polls for completion (so we can copy it to ram) as well as the
 * length of the code (so we know how much space to allocate for it)
 *
 * void flashErase(volatile uint32_t *addr, uint32_t value, volatile uint32_t *status)
 * {
 *     *addr = value;
 *     while (*status & FLASH_SR_BSY) ;
 * }
 */
static void BOOTLOADER __attribute__((naked)) __blFlashEraseCode(uint16_t **addr, uint32_t *size)
{
    asm volatile (
        "  push {lr}          \n"
        "  bl   9f            \n"
        "  str  r1, [r0, #0]  \n" // *addr = value
        "1:                   \n"
        "  ldr  r3, [r2, #0]  \n" // r3 = *status
        "  lsls r3, #15       \n" // r3 <<= 15
        "  bmi  1b            \n" // if (r3 < 0) goto 1
        "  bx   lr            \n" // return
        "9:                   \n"
        "  bic  lr, #0x1      \n"
        "  adr  r3, 9b        \n"
        "  sub  r3, lr        \n"
        "  str  lr, [r0]      \n"
        "  str  r3, [r1]      \n"
        "  pop {pc}           \n"
    );
}

/*
 * Return the address of the write code and the length of the code
 *
 * This code needs to run out of ram and not flash since accessing flash
 * while writing to flash is undefined (best case the processor stalls, worst
 * case it starts executing garbage)
 *
 * This function is used to get a pointer to the actual code that does the
 * write and polls for completion (so we can copy it to ram) as well as the
 * length of the code (so we know how much space to allocate for it)
 *
 * void flashWrite(volatile uint8_t *addr, uint8_t value, volatile uint32_t *status)
 * {
 *     *addr = value;
 *     while (*status & FLASH_SR_BSY) ;
 * }
 */
static void BOOTLOADER __attribute__((naked)) __blFlashWriteCode(uint16_t **addr, uint32_t *size)
{
    asm volatile (
        "  push {lr}          \n"
        "  bl   9f            \n"
        "  strb r1, [r0, #0]  \n" // *addr = value
        "1:                   \n"
        "  ldr  r3, [r2, #0]  \n" // r3 = *status
        "  lsls r3, #15       \n" // r3 <<= 15
        "  bmi  1b            \n" // if (r3 < 0) goto 1
        "  bx   lr            \n" // return
        "9:                   \n"
        "  bic  lr, #0x1      \n"
        "  adr  r3, 9b        \n"
        "  sub  r3, lr        \n"
        "  str  lr, [r0]      \n"
        "  str  r3, [r1]      \n"
        "  pop {pc}           \n"
    );
}

static int BOOTLOADER __blProgramFlash(uint8_t *dst, uint8_t *src,
        uint32_t length, uint32_t key1, uint32_t key2)
{
    struct StmFlash *flash = (struct StmFlash *)FLASH_BASE;
    const uint32_t sector_cnt = sizeof(__blFlashTable) / sizeof(struct blFlashTable);
    uint8_t erase_mask[sector_cnt];
    uint8_t erase_cnt;
    int offset, i, j;
    uint8_t *ptr;
    uint32_t acr_cache, cr_cache;
    uint32_t int_state = 0;

    if (((length == 0)) ||
        ((0xFFFFFFFF - (uint32_t)dst) < (length - 1)) ||
        ((dst < __blFlashTable[0].address)) ||
        ((dst + length) > (__blFlashTable[sector_cnt-1].address +
                           __blFlashTable[sector_cnt-1].length))) {
        return -1;
    }

    // disable interrupts
    // otherwise an interrupt during flash write/erase will stall the processor
    // until the write/erase completes
    int_state = __blDisableInts();

    // figure out which (if any) blocks we have to erase
    for (i=0; i<sector_cnt; i++)
        erase_mask[i] = 0;

    // compute which flash block we are starting from
    for (i=0; i<sector_cnt; i++) {
        if (dst >= __blFlashTable[i].address &&
            dst < (__blFlashTable[i].address + __blFlashTable[i].length)) {
            break;
        }
    }

    // now loop through all the flash blocks and see if we have to do any
    // 0 -> 1 transitions of a bit. If so, we must erase that block
    // 1 -> 0 transitions of a bit do not require an erase
    offset = (uint32_t)(dst - __blFlashTable[i].address);
    ptr = __blFlashTable[i].address;
    j = 0;
    erase_cnt = 0;
    while (j<length && i<sector_cnt) {
        if (offset == __blFlashTable[i].length) {
            i ++;
            offset = 0;
            ptr = __blFlashTable[i].address;
        }

        if ((ptr[offset] & src[j]) != src[j]) {
            erase_mask[i] = 1;
            erase_cnt ++;
            j += __blFlashTable[i].length - offset;
            offset = __blFlashTable[i].length;
        } else {
            j ++;
            offset ++;
        }
    }

    // wait for flash to not be busy (should never be set at this point)
    while (flash->SR & FLASH_SR_BSY) ;

    cr_cache = flash->CR;

    if (flash->CR & FLASH_CR_LOCK) {
        // unlock flash
        flash->KEYR = key1;
        flash->KEYR = key2;
    }

    if (flash->CR & FLASH_CR_LOCK) {
        // unlock failed, restore interrupts
        __blRestoreInts(int_state);

        return -1;
    }

    flash->CR = FLASH_CR_PSIZE(FLASH_CR_PSIZE_8);

    acr_cache = flash->ACR;

    // disable and flush data and instruction caches
    flash->ACR &= ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    flash->ACR |= (FLASH_ACR_DCRST | FLASH_ACR_ICRST);

    if (erase_cnt > 0)
        __blEraseSectors(sector_cnt, erase_mask);

    __blWriteBytes(dst, src, length);

    flash->ACR = acr_cache;
    flash->CR = cr_cache;

    __blRestoreInts(int_state);

    return __blCompareBytes(dst, src, length);
}

static void BOOTLOADER __blEraseSectors(uint32_t sector_cnt, uint8_t *erase_mask)
{
    struct StmFlash *flash = (struct StmFlash *)FLASH_BASE;
    int i;
    uint16_t *code_src, *code;
    uint32_t code_length;
    flashErase func;

    __blFlashEraseCode(&code_src, &code_length);

    if (code_length < BL_MAX_FLASH_CODE) {
        code = (uint16_t *)(((uint32_t)alloca(code_length+1) + 1) & ~0x1);
        func = (flashErase)((uint8_t *)code+1);

        for (i=0; i<code_length/sizeof(uint16_t); i++)
            code[i] = code_src[i];

        for (i=0; i<sector_cnt; i++) {
            if (erase_mask[i]) {
                flash->CR = (flash->CR & ~(FLASH_CR_SNB_MASK)) |
                    FLASH_CR_SNB(i) | FLASH_CR_SER;
                func(&flash->CR, flash->CR | FLASH_CR_STRT, &flash->SR);
                flash->CR &= ~(FLASH_CR_SNB_MASK | FLASH_CR_SER);
            }
        }
    }
}

static void BOOTLOADER __blWriteBytes(uint8_t *dst, uint8_t *src, uint32_t length)
{
    struct StmFlash *flash = (struct StmFlash *)FLASH_BASE;
    int i;
    uint16_t *code_src, *code;
    uint32_t code_length;
    flashWrite func;

    __blFlashWriteCode(&code_src, &code_length);

    if (code_length < BL_MAX_FLASH_CODE) {
        code = (uint16_t *)(((uint32_t)alloca(code_length+1) + 1) & ~0x1);
        func = (flashWrite)((uint8_t *)code+1);

        for (i=0; i<code_length/sizeof(uint16_t); i++)
            code[i] = code_src[i];

        flash->CR |= FLASH_CR_PG;

        for (i=0; i<length; i++) {
            if (dst[i] != src[i])
                func(&dst[i], src[i], &flash->SR);
        }

        flash->CR &= ~FLASH_CR_PG;
    }
}

static int BOOTLOADER __blCompareBytes(uint8_t *dst, uint8_t *src, uint32_t length)
{
    int i;

    for (i=0; i<length; i++) {
        if (dst[i] != src[i])
            return -1;
    }

    return 0;
}

static int BOOTLOADER __blProgramShared(uint8_t *dst, uint8_t *src,
        uint32_t length, uint32_t key1, uint32_t key2)
{
    int i;
    const int sector_cnt = sizeof(__blFlashTable) / sizeof(struct blFlashTable);

    for (i=0; i<sector_cnt; i++) {
        if ((dst >= __blFlashTable[i].address &&
             dst < (__blFlashTable[i].address + __blFlashTable[i].length)) ||
            (dst < __blFlashTable[i].address &&
             (dst + length > __blFlashTable[i].address))) {
            if (__blFlashTable[i].type != BL_FLASH_SHARED)
                return -1;
        }
    }

    return __blProgramFlash(dst, src, length, key1, key2);
}

static int BOOTLOADER __blEraseShared(uint32_t key1, uint32_t key2)
{
    struct StmFlash *flash = (struct StmFlash *)FLASH_BASE;
    const int sector_cnt = sizeof(__blFlashTable) / sizeof(struct blFlashTable);
    uint8_t erase_mask[sector_cnt];
    int erase_cnt = 0;
    int i;
    uint32_t acr_cache, cr_cache;
    uint32_t int_state = 0;

    for (i=0; i<sector_cnt; i++) {
        if (__blFlashTable[i].type == BL_FLASH_SHARED) {
            erase_mask[i] = 1;
            erase_cnt ++;
        } else {
            erase_mask[i] = 0;
        }
    }

    // disable interrupts
    // otherwise an interrupt during flash write/erase will stall the processor
    // until the write/erase completes
    int_state = __blDisableInts();

    // wait for flash to not be busy (should never be set at this point)
    while (flash->SR & FLASH_SR_BSY) ;

    cr_cache = flash->CR;

    if (flash->CR & FLASH_CR_LOCK) {
        // unlock flash
        flash->KEYR = key1;
        flash->KEYR = key2;
    }

    if (flash->CR & FLASH_CR_LOCK) {
        // unlock failed, restore interrupts
        __blRestoreInts(int_state);

        return -1;
    }

    flash->CR = FLASH_CR_PSIZE(FLASH_CR_PSIZE_8);

    acr_cache = flash->ACR;

    // disable and flush data and instruction caches
    flash->ACR &= ~(FLASH_ACR_DCEN | FLASH_ACR_ICEN);
    flash->ACR |= (FLASH_ACR_DCRST | FLASH_ACR_ICRST);

    if (erase_cnt > 0)
        __blEraseSectors(sector_cnt, erase_mask);

    flash->ACR = acr_cache;
    flash->CR = cr_cache;

    // restore interrupts
    __blRestoreInts(int_state);

    return erase_cnt;
}

static uint32_t BOOTLOADER __blCrcCont(uint8_t *addr, unsigned int length)
{
    struct StmCrc *crc = (struct StmCrc *)CRC_BASE;
    uint32_t word;
    int i;

    if ((uint32_t)addr & 0x3)
        return 0xFFFFFFFF;

    for (i=0; i<(length>>2); i++)
        crc->DR = ((uint32_t *)addr)[i];

    if (length & 0x3) {
        for (i*=4, word=0; i<length; i++)
            word |= addr[i] << ((i & 0x3) * 8);
        crc->DR = word;
    }

    return crc->DR;
}

static uint32_t BOOTLOADER __blCrc(uint8_t *addr, unsigned int length)
{
    struct StmCrc *crc = (struct StmCrc *)CRC_BASE;

    crc->CR = CRC_CR_RESET;

    return __blCrcCont(addr, length);
}

static void BOOTLOADER __blSpurious(void)
{
    //BAD!
    __blReboot();
}

static const uint32_t *__blGetPubKeysInfo(uint32_t *numKeys)
{
    extern uint32_t __pubkeys_start[];
    extern uint32_t __pubkeys_end[];
    uint32_t numWords = __pubkeys_end - __pubkeys_start;

    if (numWords % RSA_LIMBS) // something is wrong
        return NULL;

    *numKeys = numWords / RSA_LIMBS;
    return __pubkeys_start;
}

static const uint32_t* __blSigPaddingVerify(const uint32_t *rsaResult)
{
    uint32_t i;

    //all but first and last word of padding MUST have no zero bytes
    for (i = SHA2_HASH_WORDS + 1; i < RSA_WORDS - 1; i++) {
        if (!(uint8_t)(rsaResult[i] >>  0))
            return NULL;
        if (!(uint8_t)(rsaResult[i] >>  8))
            return NULL;
        if (!(uint8_t)(rsaResult[i] >> 16))
            return NULL;
        if (!(uint8_t)(rsaResult[i] >> 24))
            return NULL;
    }

    //first padding word must have all nonzero bytes except low byte
    if ((rsaResult[SHA2_HASH_WORDS] & 0xff) || !(rsaResult[SHA2_HASH_WORDS] & 0xff00) || !(rsaResult[SHA2_HASH_WORDS] & 0xff0000) || !(rsaResult[SHA2_HASH_WORDS] & 0xff000000))
        return NULL;

    //last padding word must have 0x0002 in top 16 bits and nonzero random bytes in lower bytes
    if ((rsaResult[RSA_WORDS - 1] >> 16) != 2)
        return NULL;
    if (!(rsaResult[RSA_WORDS - 1] & 0xff00) || !(rsaResult[RSA_WORDS - 1] & 0xff))
        return NULL;

    return rsaResult;
}

const struct BlVecTable __attribute__((section(".blvec"))) __BL_VECTORS =
{
    /* cortex */
    .blStackTop = (uint32_t)&__stack_top,
    .blEntry = &__blEntry,
    .blNmiHandler = &__blSpurious,
    .blMmuFaultHandler = &__blSpurious,
    .blBusFaultHandler = &__blSpurious,
    .blUsageFaultHandler = &__blSpurious,

    /* api */
    .blGetVersion = &__blVersion,
    .blReboot = &__blReboot,
    .blGetSnum = &__blGetSnum,
    .blProgramShared = &__blProgramShared,
    .blEraseShared = &__blEraseShared,
    .blGetPubKeysInfo = &__blGetPubKeysInfo,
    .blRsaPubOpIterative = &_rsaPubOpIterative,
    .blSha2init = &_sha2init,
    .blSha2processBytes = &_sha2processBytes,
    .blSha2finish = &_sha2finish,
    .blAesInitForEncr = &_aesInitForEncr,
    .blAesInitForDecr = &_aesInitForDecr,
    .blAesEncr = &_aesEncr,
    .blAesDecr = &_aesDecr,
    .blAesCbcInitForEncr = &_aesCbcInitForEncr,
    .blAesCbcInitForDecr = &_aesCbcInitForDecr,
    .blAesCbcEncr = &_aesCbcEncr,
    .blAesCbcDecr = &_aesCbcDecr,
    .blSigPaddingVerify = &__blSigPaddingVerify,
};

