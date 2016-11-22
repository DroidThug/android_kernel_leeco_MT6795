#
# Makefile for USB Host Lib
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/usb/host
ccflags-y += -I$(srctree)/drivers/usb/core

ifeq ($(CONFIG_USB_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-y	+= xhci-mtk-power.o xhci-mtk-scheduler.o xhci-mtk.o xhci-mtk-attrs.o
obj-$(CONFIG_USB_FUSB3000) += fusb3000.o
obj-$(CONFIG_USB_TUSB320) += tusb320.o pi5usb320.o
