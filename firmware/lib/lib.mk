################################################################################
#
# Nanoapp Libc/Libm build helper script
#
################################################################################

# Libm #########################################################################

LIBM_PATH := $(NANOHUB_DIR)/lib/libm

CFLAGS += -D_IEEE_LIBM

SRCS += $(LIBM_PATH)/ef_atan2.c
SRCS += $(LIBM_PATH)/ef_asin.c
SRCS += $(LIBM_PATH)/ef_fmod.c
SRCS += $(LIBM_PATH)/ef_rem_pio2.c
SRCS += $(LIBM_PATH)/ef_sqrt.c
SRCS += $(LIBM_PATH)/kf_cos.c
SRCS += $(LIBM_PATH)/kf_rem_pio2.c
SRCS += $(LIBM_PATH)/kf_sin.c
SRCS += $(LIBM_PATH)/sf_atan.c
SRCS += $(LIBM_PATH)/sf_cos.c
SRCS += $(LIBM_PATH)/sf_floor.c
SRCS += $(LIBM_PATH)/sf_fpclassify.c
SRCS += $(LIBM_PATH)/sf_round.c
SRCS += $(LIBM_PATH)/sf_scalbn.c
SRCS += $(LIBM_PATH)/sf_sin.c
SRCS += $(LIBM_PATH)/wf_atan2.c
SRCS += $(LIBM_PATH)/wf_asin.c
SRCS += $(LIBM_PATH)/wf_fmod.c

# Libc #########################################################################

LIBC_PATH := $(NANOHUB_DIR)/lib/libc

SRCS += $(LIBC_PATH)/bcopy.c
SRCS += $(LIBC_PATH)/memcmp.c
#SRCS += $(LIBC_PATH)/memcpy-armv7m.S
SRCS += $(LIBC_PATH)/memcpy.c
SRCS += $(LIBC_PATH)/memmove.c
SRCS += $(LIBC_PATH)/memset.c
SRCS += $(LIBC_PATH)/strcasecmp.c
SRCS += $(LIBC_PATH)/strlen.c
