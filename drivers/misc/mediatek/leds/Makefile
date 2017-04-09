include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

# ll85 project
	obj-$(CONFIG_MTK_AW2013_LEDS) += aw2013_leds.o

# ll96 project
	obj-$(CONFIG_MTK_LM3533_LEDS) += lm3533-i2c.o lm3533-core.o lm3533-ctrlbank.o leds-lm3533.o lm3533_bl.o

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

