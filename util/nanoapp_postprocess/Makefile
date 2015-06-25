APP = nanoapp_postprocess
SRC = postprocess.c
CC ?= gcc

$(APP): $(SRC) Makefile
	$(CC) -o $(APP) -O2 $(SRC)

clean:
	rm $(APP)
