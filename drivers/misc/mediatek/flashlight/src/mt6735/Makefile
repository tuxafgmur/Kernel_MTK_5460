#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= kd_flashlightlist.o

obj-y				+= strobe_main_sid2_part2.o
obj-y				+= strobe_sub_sid2_part1.o
obj-y				+= strobe_sub_sid2_part2.o
obj-y				+= strobe_main_sid1_part2.o
obj-y				+= strobe_part_id.o
obj-y				+= strobe_sub_sid1_part2.o

ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/flashlight/strobe_main_sid2_part1.c),)
obj-y				+= strobe_main_sid2_part1.o
endif

ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/flashlight),)
custom_kernel_flashlight := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_FLASHLIGHT)))
obj-y += $(custom_kernel_flashlight)
obj-y				+= sub_strobe.o
endif


