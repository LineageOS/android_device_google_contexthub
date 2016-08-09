PLATFORM?=stm32
CPU?=cortexm4
CHIP?=stm32f411

# SRC_PATH points to "firmware"; TOP_PATH is the abs path to top of andrlid tree
TOP_RELPATH := ../../../..
TOP_ABSPATH := $(realpath $(SRC_PATH)/$(TOP_RELPATH))
VARIANT_ABSPATH := $(TOP_ABSPATH)/$(VARIANT_PATH)
VARIANT_PATH := $(TOP_RELPATH)/$(VARIANT_PATH)

ifndef OUT
OUT:=out/nanohub/$(VARIANT)
MAKE_OUT=$(VARIANT_PATH)/$(OUT)
else
ifneq ($(filter $(TOP_ABSPATH)/out/target/product/%,$(OUT)),)
# this looks like Android OUT env var; update it
OUT:=$(OUT)/nanohub/$(VARIANT)
IMAGE_TARGET_OUT:=vendor/firmware/nanohub.full.bin
endif
MAKE_OUT:=$(OUT)
endif

.PHONY: all clean sync

all:
	make -C $(SRC_PATH) -f firmware.mk VARIANT=$(VARIANT) VARIANT_PATH=$(VARIANT_PATH) OUT=$(MAKE_OUT) PLATFORM=$(PLATFORM) CPU=$(CPU) CHIP=$(CHIP) $(EXTRA_ARGS)
ifdef IMAGE_OUT
	cd $(VARIANT_ABSPATH) && cp $(OUT)/full.bin $(IMAGE_OUT)
endif
ifdef IMAGE_TARGET_OUT
	cd $(VARIANT_ABSPATH) && mkdir -p $(dir $(TOP_ABSPATH)/$(IMAGE_TARGET_OUT)) && cp $(OUT)/full.bin $(TOP_ABSPATH)/$(IMAGE_TARGET_OUT)
endif

clean:
	rm -rf $(OUT)
ifdef IMAGE_OUT
	rm $(VARIANT_ABSPATH)/$(IMAGE_OUT)
endif
ifdef IMAGE_TARGET_OUT
	rm $(TOP_ABSPATH)/$(IMAGE_TARGET_OUT)
endif

sync:
	adb push $(OUT)/full.bin /vendor/firmware/nanohub.full.bin
