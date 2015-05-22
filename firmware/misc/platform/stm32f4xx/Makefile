M3DEBUG ?= m3debug

ifneq ($(CPU),cortexm4f)
        $(error "stm32f4xx cplatform only supports Cortex-M4F CPUs")
endif

DELIVERABLES = $(APP).full.bin $(APP).update.bin

FLAGS += -I. -fno-unwind-tables -fstack-reuse=all -ffunction-sections -fdata-sections
FLAGS += -Wl,--gc-sections
FLAGS += -Iinc/platform/$(PLATFORM)/cmsis

#platform bootloader
SRCS += src/platform/$(PLATFORM)/bl.c

#platform runtime
SRCS +=  src/platform/$(PLATFORM)/crt_$(PLATFORM).c

#platform drivers
SRCS += src/platform/$(PLATFORM)/platform.c \
	src/platform/$(PLATFORM)/usart.c \
	src/platform/$(PLATFORM)/gpio.c \
	src/platform/$(PLATFORM)/pwr.c \
	src/platform/$(PLATFORM)/i2c.c \
	src/platform/$(PLATFORM)/exti.c \
	src/platform/$(PLATFORM)/syscfg.c \
	src/platform/$(PLATFORM)/spi.c \
	src/platform/$(PLATFORM)/rtc.c \
	src/platform/$(PLATFORM)/eventQ.c \
	src/platform/$(PLATFORM)/mpu.c \
	src/platform/$(PLATFORM)/dma.c \
	src/platform/$(PLATFORM)/crc.c \
	src/platform/$(PLATFORM)/hostIntf.c

#extra deps
DEPS += $(wildcard inc/platform/$(PLATFORM)/*.h)
DEPS += $(wildcard inc/platform/$(PLATFORM)/cmsis/*.h)

#platform flags
PLATFORM_HAS_HARDWARE_CRC = true
FLAGS += -DPLATFORM_HOST_INTF_I2C_BUS=0
FLAGS += -DPLATFORM_HW_VER=0

#platform-specific rules

%.full.bin : %.elf
	$(OBJCOPY) -j.bl -j .data -j .text -I elf32-littlearm -O binary $< $@

%.update.bin : %.elf
	$(OBJCOPY) -j .data -j .text -I elf32-littlearm -O binary $< $@

flash: $(APP).full.bin
	echo -e "attach\nscript misc/platform/$(PLATFORM)/m3debug.script\nflash $(APP).full.bin 08000000\n" | $(M3DEBUG)
