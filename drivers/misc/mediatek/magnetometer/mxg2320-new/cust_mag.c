//------------------------James Modified for K5.-------
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>
static struct mag_hw cust_mag_hw = {
    .i2c_num = 3,
    .i2c_addr = {0x0C,0,0,0},////0x0D----depend on Pin CAD.
    .direction = 0,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .is_batch_supported = false,
};
struct mag_hw* get_cust_mag_hw(void) 
{
//asdffffffffffrrrrt
    return &cust_mag_hw;
}
//------------------------------------------------

