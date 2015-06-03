#variant makefile for generic linux


ifneq ($(PLATFORM),linux)
        $(error "linux variant cannot be build on a platform that is not linux")
endif

FLAGS += -DHEAP_SIZE=102400           #100K heap
