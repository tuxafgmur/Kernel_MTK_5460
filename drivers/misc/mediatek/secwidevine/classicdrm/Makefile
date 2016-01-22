
ifneq ($(MTK_ROOT_BUILD),)
include $(MTK_ROOT_BUILD)/Makefile
else
include $(srctree)/drivers/misc/mediatek/Makefile.custom
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)

$(warning "Build widevine driver")
$(warning "MTK_PATH_PLATFORM=$(MTK_PATH_PLATFORM)")

ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/utils/tlutils
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 
obj-y += secwidevine.o

endif