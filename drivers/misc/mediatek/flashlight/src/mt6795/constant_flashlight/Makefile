#
# Makefile for misc devices that really don't fit anywhere else.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_ARCH_MTK_PROJECT),"x608")
ccflags-y += -DTORCH_31MA
endif

obj-y				+= leds_strobe.o
