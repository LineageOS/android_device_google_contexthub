#variant makefile for nucleo


ifneq ($(PLATFORM),stm32f4xx)
        $(error "NUCLEO variant cannot be build on a platform that is not stm32f411")
endif

FLAGS += -DHEAP_SIZE=102400           #100K heap
FLAGS += -DPLATFORM_HW_TYPE=0x4E75   #'Nu' ->nucleo
FLAGS += -DDEBUG_SWD

#board configuration shims
SRCS += src/variant/$(VARIANT)/i2c.c
SRCS += src/variant/$(VARIANT)/spi.c

#keys

#fusion algorithm
SRCS += src/fusion/fusion.c src/fusion/mat.c src/fusion/quat.c src/fusion/vec.c

#accel, gyro and mag drivers
SRCS += src/drivers/bosch_bmi160_bmm150/bosch_bmi160_bmm150.c src/drivers/bosch_bmi160_bmm150/mag_cal.c

#linker script
LKR = misc/variant/common/stm32f411.lkr
OSFLAGS += -Wl,-T $(LKR)
DEPS += $(LKR)
