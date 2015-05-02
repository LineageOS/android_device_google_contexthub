#makefile for hostinfo app

SELF_MKFILE := $(abspath $(lastword $(MAKEFILE_LIST)))
SELF_DIR := $(SELF_MKFILE:Makefile=)
SELF_FILES := $(wildcard $(SELF_DIR)*.c)

SRCS += $(SELF_FILES)
