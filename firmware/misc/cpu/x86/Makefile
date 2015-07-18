GCC = gcc
OBJCOPY = objcopy

FLAGS += -march=core2 -msse2 -DSYSCALL_VARARGS_PARAMS_PASSED_AS_PTRS

#cpu runtime
SRCS += src/cpu/$(CPU)/atomicBitset.c src/cpu/$(CPU)/cpu.c src/cpu/$(CPU)/atomic.c

#extra deps
DEPS += $(wildcard inc/cpu/$(CPU)/*.h)


