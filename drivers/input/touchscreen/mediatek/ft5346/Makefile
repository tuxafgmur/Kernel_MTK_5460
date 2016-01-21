include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/mt6735/$(ARCH_MTK_PROJECT)/touchpanel/ft5346/

ifeq ($(CONFIG_MTK_PLATFORM),"mt6572")
    ccflags-y += -DMT6572
endif

obj-y	+=  focaltech_ctl.o
obj-y	+=  ft6x06_button.o
obj-y	+=  ft6x06_driver.o
obj-y	+=  ft6x06_iic_wrap.o
obj-y	+=  ft6x06_isp.o

