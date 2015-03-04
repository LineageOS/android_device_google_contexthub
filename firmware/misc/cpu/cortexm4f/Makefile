GCC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS += -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mno-thumb-interwork

#cpu runtime
SRCS += src/cpu/$(CPU)/atomicBitset.c src/cpu/$(CPU)/atomicXchg.c src/cpu/$(CPU)/trylock.c src/cpu/$(CPU)/cpu.c

#extra deps
DEPS += $(wildcard inc/cpu/$(CPU)/*.h)


