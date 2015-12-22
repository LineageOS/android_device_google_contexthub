GCC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS += -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mno-thumb-interwork -ffast-math -fsingle-precision-constant -DARM -DMY_FLOAT_RUNTIME_SUCKS
APPFLAGS += -Wl,-T misc/cpu/$(CPU)/app.lkr -mno-pic-data-is-text-relative

#cpu runtime
SRCS += src/cpu/$(CPU)/atomicBitset.c src/cpu/$(CPU)/cpu.c src/cpu/$(CPU)/pendsv.c src/cpu/$(CPU)/atomic.c src/cpu/$(CPU)/syscall.c src/cpu/$(CPU)/appSupport.c

#extra deps
DEPS += $(wildcard inc/cpu/$(CPU)/*.h)

#bad words for C-M4F
BADWORDS += "__floatundisf=When casting a uint64_t to float, use floatFromUint64"
BADWORDS += "__floatdisf=When casting a int64_t to float, use floatFromInt64"
BADWORDS += "__fixunssfdi=When casting a float to a uint64_t, use floatToUint64"
BADWORDS += "__fixsfdi=When casting a float to a int64_t, use floatToInt64"
#BADWORDS += "__aeabi_uldivmod=Do not ever divide uint64_t by anything"
#BADWORDS += "__aeabi_ldivmod=Do not ever divide int64_t by anything"




