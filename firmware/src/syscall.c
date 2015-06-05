#include <syscall.h>
#include <stdio.h>



static uint32_t mTableStore[(sizeof(struct SyscallTable) + sizeof(union SyscallTableEntry[1 << SYSCALL_BITS_LEVEL_0]) + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
static const uint8_t mLevelBits[] = {SYSCALL_BITS_LEVEL_0, SYSCALL_BITS_LEVEL_1, SYSCALL_BITS_LEVEL_2, SYSCALL_BITS_LEVEL_3, 0};
static struct SyscallTable *mTable = (struct SyscallTable*)mTableStore;



void syscallInit(void)
{
    mTable->numEntries = 1 << SYSCALL_BITS_LEVEL_0;
}

bool syscallAddTable(uint32_t path, uint32_t level, struct SyscallTable *table)
{
    struct SyscallTable** tabP = &mTable;
    const uint8_t *bits = mLevelBits;

    while (level--) {
        uint32_t idx = path >> (32 - *bits);
        path <<= *bits++;

        //cannot traverse
        if ((*tabP)->numEntries <= idx)
            return false;

        //cannot add table as final leaf
        if (*bits == 0)
            return false;

        tabP = &(*tabP)->entry[idx].subtable;
    }

    *tabP = table;
    return true;
}

static SyscallFunc* syscallFindHandlerLoc(uint32_t path)
{
    struct SyscallTable* tab = mTable;
    const uint8_t *bits = mLevelBits;

    while (tab) {
        uint32_t idx = path >> (32 - *bits);

        path <<= *bits++;

        if (tab->numEntries <= idx)
            break;

        if (*bits)
            return &tab->entry[idx].func;
        else
            tab = tab->entry[idx].subtable;
    }

    return NULL;
}

bool syscallAddFunc(uint32_t path, SyscallFunc func)
{
    SyscallFunc *f = syscallFindHandlerLoc(path);

    if (!f)
        return false;

    *f = func;
    return true;
}

SyscallFunc syscallGetHandler(uint32_t path)
{
    SyscallFunc *f = syscallFindHandlerLoc(path);

    return f ? *f : NULL;
}


