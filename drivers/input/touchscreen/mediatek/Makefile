include $(srctree)/drivers/misc/mediatek/Makefile.custom


# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

ifeq ($(CONFIG_TINNO_TGESTURE),y)
obj-y	+=  tgesture/
endif

ifeq ($(CONFIG_MTK_FT5346),y)
obj-y	+=  ft5346/
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS),y)
obj-y   +=  synaptics/
endif
