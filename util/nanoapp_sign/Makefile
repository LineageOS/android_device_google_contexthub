APP = nanoapp_sign
SRC = nanoapp_sign.c ../../firmware/src/rsa.c ../../firmware/src/sha2.c
CC ?= gcc

$(APP): $(SRC) Makefile
	$(CC) -o $(APP) -O2 $(SRC) -I../../firmware/inc -DRSA_SUPPORT_PRIV_OP_BIGRAM

clean:
	rm $(APP)
