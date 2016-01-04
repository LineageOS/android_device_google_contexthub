#
# Copyright (C) 2016 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#variant makefile for lunchbox

ifneq ($(PLATFORM),stm32f4xx)
        $(error "LUNCHBOX variant cannot be build on a platform that is not stm32f411")
endif

FLAGS += -DHEAP_SIZE=112640          #110K heap
FLAGS += -DPLATFORM_HW_TYPE=0x4C75   #'Lu' -> lunchbox
FLAGS += -DDEBUG_UART_PIN=0          #GPIOA0 is debug uart at 2MBps

#board configuration shims
SRCS += links/variant/src/i2c.c
SRCS += links/variant/src/spi.c

#keys

#drivers
# ROHM ALS/prox
SRCS += src/drivers/rohm_rpr0521/rohm_rpr0521.c

# Fusion algorithms
SRCS += src/fusion/fusion.c src/fusion/mat.c src/fusion/quat.c src/fusion/vec.c

# BMI160 accel and gyro, BMM150 mag drivers
SRCS += src/drivers/bosch_bmi160_bmm150/bosch_bmi160_bmm150.c src/drivers/bosch_bmi160_bmm150/mag_cal.c src/drivers/bosch_bmi160_bmm150/time_sync.c

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

DEPS += $(wildcard links/variant/inc/*.h)

#linker script
LKR = misc/variant/common/stm32f411.lkr
OSFLAGS += -Wl,-T $(LKR)
DEPS += $(LKR)
