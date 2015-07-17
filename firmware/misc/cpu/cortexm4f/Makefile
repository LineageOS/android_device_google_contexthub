GCC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS += -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mno-thumb-interwork -ffast-math -fsingle-precision-constant
APPFLAGS += -Wl,-T misc/cpu/$(CPU)/app.lkr -mno-pic-data-is-text-relative

#cpu runtime
SRCS += src/cpu/$(CPU)/atomicBitset.c src/cpu/$(CPU)/cpu.c src/cpu/$(CPU)/pendsv.c src/cpu/$(CPU)/atomic.c src/cpu/$(CPU)/syscall.c

#extra deps
DEPS += $(wildcard inc/cpu/$(CPU)/*.h)


