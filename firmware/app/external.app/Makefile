#makefile for all extrernal appspop app

EXTRA_FLAGS ?= 

ifneq ($(EXTERNAL_APP_BUILD),)
 SELF_MKFILE := $(EXTERNAL_APP_MAKEFILE)
 APP_ID := $(EXTERNAL_APP_ID)
 include app/common.mk
endif

