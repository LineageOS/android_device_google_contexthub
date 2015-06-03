DELIVERABLES = $(APP).bin
LKR = misc/platform/$(PLATFORM)/linux.extra.lkr

FLAGS += -I. -fno-unwind-tables -fstack-reuse=all -ffunction-sections -fdata-sections -m32
FLAGS += -Wl,-T $(LKR) -Wl,--gc-sections


#platform drivers
SRCS += src/platform/$(PLATFORM)/platform.c \
	src/platform/$(PLATFORM)/i2c.c \
	src/platform/$(PLATFORM)/spi.c \
	src/platform/$(PLATFORM)/rtc.c \
	src/platform/$(PLATFORM)/eventQ.c \
	src/platform/$(PLATFORM)/hostIntf.c

#extra deps
DEPS += $(wildcard inc/platform/$(PLATFORM)/*.h)
DEPS += $(LKR)

#platform flags
FLAGS += -DPLATFORM_HOST_INTF_I2C_BUS=0
FLAGS += -DPLATFORM_HW_TYPE=0x8086 -DPLATFORM_HW_VER=0
FLAGS += -DPLAT_HAS_NO_U_TYPES_H

#platform-specific rules

%.bin : %.elf
	mv $< $@

