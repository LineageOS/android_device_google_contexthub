PLATFORM ?= stm32f411
CPU ?= cortexm4f
APP = nanohub

MAKE_PLAT = misc/platform/$(PLATFORM)/Makefile
MAKE_CPU = misc/cpu/$(CPU)/Makefile

real: all

include $(MAKE_PLAT)
include $(MAKE_CPU)

FLAGS += -O2 -Wall -Werror -Iinc -Ilinks -g -ggdb3
#help avoid commmon embedded C mistakes
FLAGS += -Wmissing-declarations -Wlogical-op -Waddress -Wempty-body -Wpointer-arith -Wenum-compare -Wdouble-promotion -Wfloat-equal -Wshadow

#debug mode
FLAGS += -DDEBUG

#frameworks
SRCS += src/printf.c src/timer.c src/seos.c src/heap.c src/slab.c src/spi.c src/trylock.c 

#app code
SRCS += app/test_app0.c app/test_app1.c


#extra deps
DEPS += $(wildcard app/*.h)
DEPS += $(wildcard inc/*.h)
DEPS += Makefile $(MAKE_PLAT) $(MAKE_CPU)


all: symlinks $(DELIVERABLES)

symlinks: links/p_$(PLATFORM) links/c_$(CPU)

links/p_$(PLATFORM):
	rm -rf links/plat
	mkdir -p links/plat
	ln -s ../../misc/platform/$(PLATFORM) links/plat/misc
	ln -s ../../src/platform/$(PLATFORM) links/plat/src
	ln -s ../../inc/platform/$(PLATFORM) links/plat/inc
	touch links/p_$(PLATFORM)

links/c_$(CPU):
	rm -rf links/cpu
	mkdir -p links/cpu
	ln -s ../../misc/cpu/$(CPU) links/cpu/misc
	ln -s ../../src/cpu/$(CPU) links/cpu/src
	ln -s ../../inc/cpu/$(CPU) links/cpu/inc
	touch links/c_$(CPU)

$(APP).elf: symlinks $(SRCS) $(DEPS)
	$(GCC) -o $(APP).elf $(FLAGS) $(SRCS)

clean:
	rm -rf $(DELIVERABLES) $(APP).elf links

