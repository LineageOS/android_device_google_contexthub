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

#drivers
# AMS ALS/prox
SRCS += src/drivers/ams_tmd2772/ams_tmd2772.c

# ROHM ALS/prox
SRCS += src/drivers/rohm_rpr0521/rohm_rpr0521.c

# Fusion algorithm
SRCS += src/fusion/fusion.c src/fusion/mat.c src/fusion/quat.c src/fusion/vec.c

# BMI160 accel and gyro, BMM150 mag drivers
SRCS += src/drivers/bosch_bmi160_bmm150/bosch_bmi160_bmm150.c src/drivers/bosch_bmi160_bmm150/mag_cal.c

# Orientation sensor driver
SRCS += src/drivers/orientation/orientation.c

# Window orientation sensor driver
SRCS += src/drivers/window_orientation/window_orientation.c

# Bosch BMP280 Barometer/Temperature
SRCS += src/drivers/bosch_bmp280/bosch_bmp280.c

# Hall effect sensor driver
SRCS += src/drivers/hall/hall.c

# Camera Vsync driver
SRCS += src/drivers/vsync/vsync.c

# Tilt detection
SRCS += src/drivers/tilt_detection/tilt_detection.c

#linker script
LKR = misc/variant/common/stm32f411.lkr
OSFLAGS += -Wl,-T $(LKR)
DEPS += $(LKR)
