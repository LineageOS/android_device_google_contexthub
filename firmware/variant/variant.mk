PLATFORM?=stm32
CPU?=cortexm4
CHIP?=stm32f411
OUT?=out

# SRC_PATH points to "firmware"; TOP_PATH is the abs path to top of andrlid tree
TOP_RELPATH := ../../../..
TOP_ABSPATH := $(realpath $(SRC_PATH)/$(TOP_RELPATH))
VARIANT_ABSPATH := $(TOP_ABSPATH)/$(VARIANT_PATH)
VARIANT_PATH := $(TOP_RELPATH)/$(VARIANT_PATH)

all:
	make -C $(SRC_PATH) -f firmware.mk VARIANT=$(VARIANT) VARIANT_PATH=$(VARIANT_PATH) OUT=$(VARIANT_PATH)/$(OUT) PLATFORM=$(PLATFORM) CPU=$(CPU) CHIP=$(CHIP) $(EXTRA_ARGS)
ifdef IMAGE_OUT
	cd $(VARIANT_ABSPATH) && cp $(OUT)/full.bin $(IMAGE_OUT)
endif

clean:
	rm -rf $(OUT)
ifdef IMAGE_OUT
	rm $(IMAGE_OUT)
endif
