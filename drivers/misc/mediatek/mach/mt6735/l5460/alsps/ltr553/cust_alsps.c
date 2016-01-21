#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},

    /* MTK: modified to support AAL */
    .als_level  = { 2,  16,  32, 64,  613, 1400,  2238,  3253,  4401, 5606, 6844, 8196, 65535, 65535, 65535},
   // .als_value  = { 20,  50,  95,  190,  1250,  1700,  1920,  2900, 5745, 8500, 10243, 10243, 10243, 10243, 10243},
    .als_value  = { 18,  50,  95,  190,  1000,  1700,  1920,  2900, 5745, 8500, 10243, 10243, 10243, 10243, 10243},
    .ps_threshold = 2,  //3
    .ps_threshold_high = 0x167,
    .ps_threshold_low = 0x74,
    .als_threshold_high = 0xFFFF,
    .als_threshold_low = 0,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

