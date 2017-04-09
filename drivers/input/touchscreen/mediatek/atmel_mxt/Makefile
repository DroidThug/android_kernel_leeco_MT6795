include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/atmel_mxt/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

ifeq ($(TARGET_BUILD_VARIANT), eng)
    ccflags-y += -DBUILD_DEBUG=1
else
    ccflags-y += -DBUILD_DEBUG=0
endif

obj-y	+=  atmel_mxt_ts_letv.o
