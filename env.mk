ifeq ($(glibc), 1)
CROSS_COMPILE=$(TOP_DIR)/env/buildroot-2016.05-rc2/output/host/usr/bin/arm-linux-
else
CROSS_COMPILE=arm-linux-
endif
CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar

export CC AR 

