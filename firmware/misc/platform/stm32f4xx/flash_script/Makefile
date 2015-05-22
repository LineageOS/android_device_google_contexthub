GCC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

FLAGS += -mthumb -mcpu=cortex-m4 -march=armv7e-m -mno-thumb-interwork -fno-unwind-tables -fstack-reuse=all -Os -Wall -Werror -nostartfiles


tool.bin: tool.elf Makefile
	$(OBJCOPY) -j .data -j .text -I elf32-littlearm -O binary tool.elf tool.bin

tool.elf: tool.c Makefile
	$(GCC) -o tool.elf $(FLAGS) tool.c

clean:
	rm -rf tool.o tool.bin

