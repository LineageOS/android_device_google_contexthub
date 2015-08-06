#variant makefile for angler


ifneq ($(PLATFORM),stm32f4xx)
        $(error "ANGLER variant cannot be build on a platform that is not stm32f411")
endif

FLAGS += -DHEAP_SIZE=102400           #100K heap
FLAGS += -DPLATFORM_HW_TYPE=0x416E   #'An' -> angler

#board configuration shims
SRCS += src/variant/$(VARIANT)/i2c.c
SRCS += src/variant/$(VARIANT)/spi.c

#linker script
LKR = misc/variant/common/stm32f411.lkr
OSFLAGS += -Wl,-T $(LKR)
DEPS += $(LKR)
