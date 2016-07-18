# settings that apps and OS both want to know about variant

VENDOR := google
VARIANT := lunchbox
CPU := cortexm4
CHIP := stm32f411
PLATFORM := stm32

# VARIANT_PATH is relative to ANDROID TOP
VARIANT_PATH := device/google/contexthub/firmware/variant/$(VARIANT)
