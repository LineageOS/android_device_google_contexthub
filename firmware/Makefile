#find build target
PLATFORM ?= stm32f4xx
CPU ?= cortexm4f
VARIANT ?= pike

APP = nanohub

#find makefiles
MAKE_PLAT = misc/platform/$(PLATFORM)/Makefile
MAKE_CPU = misc/cpu/$(CPU)/Makefile
MAKE_VAR = misc/variant/$(VARIANT)/Makefile

#top make target
SRCS := 
DELIVERABLES := 
real: all

#include makefiles for plat and cpu
include $(MAKE_PLAT)
include $(MAKE_CPU)
include $(MAKE_VAR)

FLAGS += -Wall -Werror -Iinc -Ilinks -Iexternal/freebsd/inc -fshort-double
#help avoid commmon embedded C mistakes
FLAGS += -Wmissing-declarations -Wlogical-op -Waddress -Wempty-body -Wpointer-arith -Wenum-compare -Wdouble-promotion -Wfloat-equal -Wshadow

OSFLAGS += -g -ggdb3 -D_OS_BUILD_ -O2 
APPFLAGS += -Os

#debug mode
FLAGS += -DDEBUG

#frameworks
SRCS += src/printf.c src/timer.c src/seos.c src/heap.c src/slab.c src/spi.c src/trylock.c
SRCS += src/hostIntf.c src/hostIntfI2c.c src/hostIntfSpi.c src/nanohubCommand.c src/sensors.c src/syscall.c

ifndef PLATFORM_HAS_HARDWARE_CRC
SRCS += src/softcrc.c
endif

#app code
include $(wildcard app/*/Makefile)


#extra deps
DEPS += $(wildcard app/*.h)
DEPS += $(wildcard inc/*.h)
DEPS += Makefile $(MAKE_PLAT) $(MAKE_CPU) $(MAKE_VAR)

all: symlinks $(DELIVERABLES)

symlinks: links/p_$(PLATFORM) links/c_$(CPU) links/v_$(VARIANT)

links/p_$(PLATFORM):
	rm -rf links/plat links/p_*
	mkdir -p links/plat
	ln -s ../../misc/platform/$(PLATFORM) links/plat/misc
	ln -s ../../src/platform/$(PLATFORM) links/plat/src
	ln -s ../../inc/platform/$(PLATFORM) links/plat/inc
	touch links/p_$(PLATFORM)

links/c_$(CPU):
	rm -rf links/cpu links/c_*
	mkdir -p links/cpu
	ln -s ../../misc/cpu/$(CPU) links/cpu/misc
	ln -s ../../src/cpu/$(CPU) links/cpu/src
	ln -s ../../inc/cpu/$(CPU) links/cpu/inc
	touch links/c_$(CPU)

links/v_$(VARIANT):
	rm -rf links/variant links/v_*
	mkdir -p links/variant
	ln -s ../../misc/variant/$(VARIANT) links/variant/misc
	ln -s ../../src/variant/$(VARIANT) links/variant/src
	ln -s ../../inc/variant/$(VARIANT) links/variant/inc
	touch links/v_$(VARIANT)

$(APP).elf: symlinks $(SRCS) $(DEPS)
	$(GCC) -o $(APP).elf $(FLAGS) $(OSFLAGS) $(SRCS)

clean:
	rm -rf $(DELIVERABLES) $(APP).elf links $(CLEANFILES)

