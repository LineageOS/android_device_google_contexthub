APP = nanoapp_encr
SRC = nanoapp_encr.c ../../firmware/src/aes.c
CC ?= gcc

$(APP): $(SRC) Makefile
	$(CC) -o $(APP) -O2 $(SRC) -I../../firmware/inc

clean:
	rm $(APP)
