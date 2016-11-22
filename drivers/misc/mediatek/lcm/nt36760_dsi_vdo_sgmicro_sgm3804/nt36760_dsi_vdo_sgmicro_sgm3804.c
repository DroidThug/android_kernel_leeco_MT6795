#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
#include <string.h>
#else

#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#endif

#include <cust_gpio_usage.h>
#include <cust_i2c.h>

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static struct incelll_wg{      //for incell module tp gesture wakeup
    void *prmi4_data;
    void (*tp_wg_func)(struct synaptics_rmi4_data *rmi4_data, bool enable);
    void (*tp_resume_func)(struct early_suspend *h);
    int wg_enable;
};
extern struct incelll_wg rmi4_tp_wg;

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_NT36760 (0x90)

//#define GPIO_SGM3804_ENN   GPIO_LCD_BIAS_ENN_PIN
//#define GPIO_SGM3804_ENP   GPIO_LCD_BIAS_ENP_PIN
#define GPIO_SGM3804_ENN   (GPIO131|0x80000000)
#define GPIO_SGM3804_ENP   (GPIO9  |0x80000000)
//#define  LCM_DSI_CMD_MODE	1
#define  LCM_DSI_CMD_MODE	0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)				lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)												lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)							lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)												lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   					lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/

#define SGM_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 2
#define I2C_ID_NAME "sgm3804"
#define SGM_ADDR 0x3E

/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata sgm3804_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, SGM_ADDR)};
static struct i2c_client *sgm3804_i2c_client = NULL;
extern void msleep(unsigned int msecs);

/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int sgm3804_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int sgm3804_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct sgm3804_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id sgm3804_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver sgm3804_iic_driver = {
	.id_table	= sgm3804_id,
	.probe		= sgm3804_probe,
	.remove		= sgm3804_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sgm3804",
	},
 
};
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int sgm3804_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "sgm3804_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	sgm3804_i2c_client = client;		
	return 0;      
}


static int sgm3804_remove(struct i2c_client *client)
{  	
  printk( "sgm3804_remove\n");
  sgm3804_i2c_client = NULL;
  i2c_unregister_device(client);
  return 0;
}


int sgm3804_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	char write_data[2]={0};
	unsigned char retry;
	write_data[0]= addr;
	write_data[1] = value;
	if(sgm3804_i2c_client == NULL)
	{
		return -1;
	}
	for (retry = 0; retry < 10; retry++) {
	ret=i2c_master_send(sgm3804_i2c_client, write_data, 2);
	if(ret<0){
	printk("sgm3804 write data fail  cmd=%0x\n",addr);
		msleep(20);
		}
	else{
		printk("sgm3804 write data success !! cmd=%0x\n",addr);
		break;
	}
		}
	return ret ;
}

EXPORT_SYMBOL(sgm3804_write_bytes);


int sgm3804_read_bytes(unsigned char addr, unsigned char *value ,unsigned char target_vol)
{	
	int ret = 0;
	char write_data[2]={0};
	unsigned char retry;
	write_data[0]= addr;
	if(sgm3804_i2c_client == NULL)
	{
		return -1;
	}

	ret=i2c_master_send(sgm3804_i2c_client, write_data, 1);
	if(ret<0){
		printk("sgm3804 send data fail  cmd=%0x\n",addr);
	}
    ret = i2c_master_recv(sgm3804_i2c_client, &write_data[1], 1);
    if (ret < 0)
    {
        printk("sgm3804 receive data fail  cmd=%0x\n",addr);
    }

	if(write_data[1]==target_vol)
	{
		printk("sgm3804 read same data, target vol : 0x%x\n",write_data[1]);
	}else{
		printk("sgm3804 read different data  cur_vol:0x%x target_val:0x%x\n",write_data[1],target_vol);
	}

	*value=write_data[1];

	//printk("sgm3804_read_bytes: ret=%d\n",ret);
	return ret ;
}

EXPORT_SYMBOL(sgm3804_read_bytes);

/*
 * module load/unload record keeping
 */

static int __init sgm3804_iic_init(void)
{

   printk( "sgm3804_iic_init\n");
   i2c_register_board_info(SGM_I2C_BUSNUM, &sgm3804_board_info, 1);
   printk( "sgm3804_iic_init2\n");
   i2c_add_driver(&sgm3804_iic_driver);
   printk( "sgm3804_iic_init success\n");	
   return 0;
}

static void __exit sgm3804_iic_exit(void)
{
  printk( "sgm3804_iic_exit\n");
  i2c_del_driver(&sgm3804_iic_driver);  
}


module_init(sgm3804_iic_init);
module_exit(sgm3804_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK sgm3804 I2C Driver");
MODULE_LICENSE("GPL"); 

#endif



#ifdef BUILD_LK

#define sgm3804_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t sgm3804_i2c;

static int sgm3804_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    sgm3804_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    sgm3804_i2c.addr = (sgm3804_SLAVE_ADDR_WRITE >> 1);
    sgm3804_i2c.mode = ST_MODE;
    sgm3804_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&sgm3804_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
	
//	#define sgm3804_write_byte(add, data)  mt8193_i2c_write(add, data)
//	#define sgm3804_read_byte(add)  mt8193_i2c_read(add)
  

#endif

static char reg_init[][2] =
{
0xFF, 0x20,
0xFB, 0x01,
0x75, 0x00,
0x76, 0x30,
0x77, 0x00,
0x78, 0x56,
0x79, 0x00,
0x7A, 0x84,
0x7B, 0x00,
0x7C, 0xA4,
0x7D, 0x00,
0x7E, 0xBC,
0x7F, 0x00,
0x80, 0xD1,
0x81, 0x00,

0x82, 0xE3,
0x83, 0x00,
0x84, 0xF4,
0x85, 0x01,
0x86, 0x02,
0x87, 0x01,
0x88, 0x33,
0x89, 0x01,
0x8A, 0x5A,
0x8B, 0x01,
0x8C, 0x94,
0x8D, 0x01,
0x8E, 0xC3,
0x8F, 0x02,
0x90, 0x09,
0x91, 0x02,
0x92, 0x41,
0x93, 0x02,
0x94, 0x43,
0x95, 0x02,
0x96, 0x73,
0x97, 0x02,
0x98, 0xAB,
0x99, 0x02,
0x9A, 0xCF,
0x9B, 0x02,
0x9C, 0xFE,
0x9D, 0x03,
0x9E, 0x1C,
0x9F, 0x03,
0xA0, 0x43,
0xA2, 0x03,
0xA3, 0x51,
0xA4, 0x03,
0xA5, 0x5D,
0xA6, 0x03,
0xA7, 0x6B,
0xA9, 0x03,
0xAA, 0x7C,
0xAB, 0x03,
0xAC, 0x8F,
0xAD, 0x03,
0xAE, 0xA1,
0xAF, 0x03,
0xB0, 0xB5,
0xB1, 0x03,
0xB2, 0xCB,
0xB3, 0x00,
0xB4, 0x30,
0xB5, 0x00,
0xB6, 0x56,
0xB7, 0x00,
0xB8, 0x84,
0xB9, 0x00,
0xBA, 0xA4,
0xBB, 0x00,
0xBC, 0xBC,
0xBD, 0x00,
0xBE, 0xD1,
0xBF, 0x00,
0xC0, 0xE3,
0xC1, 0x00,
0xC2, 0xF4,
0xC3, 0x01,
0xC4, 0x02,
0xC5, 0x01,
0xC6, 0x33,
0xC7, 0x01,
0xC8, 0x5A,
0xC9, 0x01,
0xCA, 0x94,
0xCB, 0x01,
0xCC, 0xC3,
0xCD, 0x02,
0xCE, 0x09,
0xCF, 0x02,
0xD0, 0x41,
0xD1, 0x02,
0xD2, 0x43,
0xD3, 0x02,
0xD4, 0x73,
0xD5, 0x02,
0xD6, 0xAB,
0xD7, 0x02,
0xD8, 0xCF,
0xD9, 0x02,
0xDA, 0xFE,
0xDB, 0x03,
0xDC, 0x1C,
0xDD, 0x03,
0xDE, 0x43,
0xDF, 0x03,
0xE0, 0x51,
0xE1, 0x03,
0xE2, 0x5D,
0xE3, 0x03,
0xE4, 0x6B,
0xE5, 0x03,
0xE6, 0x7C,
0xE7, 0x03,
0xE8, 0x8F,
0xE9, 0x03,
0xEA, 0xA1,
0xEB, 0x03,
0xEC, 0xB5,
0xED, 0x03,
0xEE, 0xCB,
0xEF, 0x00,
0xF0, 0x30,
0xF1, 0x00,
0xF2, 0x56,
0xF3, 0x00,
0xF4, 0x84,
0xF5, 0x00,
0xF6, 0xA4,
0xF7, 0x00,
0xF8, 0xBC,
0xF9, 0x00,
0xFA, 0xD1,

0xFF, 0x21,
0xFB, 0x01,
0x00, 0x00,
0x01, 0xE3,
0x02, 0x00,
0x03, 0xF4,
0x04, 0x01,
0x05, 0x02,
0x06, 0x01,
0x07, 0x33,
0x08, 0x01,
0x09, 0x5A,
0x0A, 0x01,
0x0B, 0x94,
0x0C, 0x01,
0x0D, 0xC3,
0x0E, 0x02,
0x0F, 0x09,
0x10, 0x02,
0x11, 0x41,
0x12, 0x02,
0x13, 0x43,
0x14, 0x02,
0x15, 0x73,
0x16, 0x02,
0x17, 0xAB,
0x18, 0x02,
0x19, 0xCF,
0x1A, 0x02,
0x1B, 0xFE,
0x1C, 0x03,
0x1D, 0x1C,
0x1E, 0x03,
0x1F, 0x43,
0x20, 0x03,
0x21, 0x51,
0x22, 0x03,
0x23, 0x5D,
0x24, 0x03,
0x25, 0x6B,
0x26, 0x03,
0x27, 0x7C,
0x28, 0x03,
0x29, 0x8F,
0x2A, 0x03,
0x2B, 0xA1,
0x2D, 0x03,
0x2F, 0xB5,
0x30, 0x03,
0x31, 0xCB,
0x32, 0x00,
0x33, 0x30,
0x34, 0x00,
0x35, 0x56,
0x36, 0x00,
0x37, 0x84,
0x38, 0x00,
0x39, 0xA4,
0x3A, 0x00,
0x3B, 0xBC,
0x3D, 0x00,
0x3F, 0xD1,
0x40, 0x00,
0x41, 0xE3,
0x42, 0x00,
0x43, 0xF4,
0x44, 0x01,
0x45, 0x02,
0x46, 0x01,
0x47, 0x33,
0x48, 0x01,
0x49, 0x5A,
0x4A, 0x01,
0x4B, 0x94,
0x4C, 0x01,
0x4D, 0xC3,
0x4E, 0x02,
0x4F, 0x09,
0x50, 0x02,
0x51, 0x41,
0x52, 0x02,
0x53, 0x43,
0x54, 0x02,
0x55, 0x73,
0x56, 0x02,
0x58, 0xAB,
0x59, 0x02,
0x5A, 0xCF,
0x5B, 0x02,
0x5C, 0xFE,
0x5D, 0x03,
0x5E, 0x1C,
0x5F, 0x03,
0x60, 0x43,
0x61, 0x03,
0x62, 0x51,
0x63, 0x03,
0x64, 0x5D,
0x65, 0x03,
0x66, 0x6B,
0x67, 0x03,
0x68, 0x7C,
0x69, 0x03,
0x6A, 0x8F,
0x6B, 0x03,
0x6C, 0xA1,
0x6D, 0x03,
0x6E, 0xB5,
0x6F, 0x03,
0x70, 0xCB,
0x71, 0x00,
0x72, 0x30,
0x73, 0x00,
0x74, 0x56,
0x75, 0x00,
0x76, 0x84,
0x77, 0x00,
0x78, 0xA4,
0x79, 0x00,
0x7A, 0xBC,
0x7B, 0x00,
0x7C, 0xD1,
0x7D, 0x00,
0x7E, 0xE3,
0x7F, 0x00,
0x80, 0xF4,
0x81, 0x01,
0x82, 0x02,
0x83, 0x01,
0x84, 0x33,
0x85, 0x01,
0x86, 0x5A,
0x87, 0x01,
0x88, 0x94,
0x89, 0x01,
0x8A, 0xC3,
0x8B, 0x02,
0x8C, 0x09,
0x8D, 0x02,
0x8E, 0x41,
0x8F, 0x02,
0x90, 0x43,
0x91, 0x02,
0x92, 0x73,
0x93, 0x02,
0x94, 0xAB,
0x95, 0x02,
0x96, 0xCF,
0x97, 0x02,
0x98, 0xFE,
0x99, 0x03,
0x9A, 0x1C,
0x9B, 0x03,
0x9C, 0x43,
0x9D, 0x03,
0x9E, 0x51,
0x9F, 0x03,
0xA0, 0x5D,
0xA2, 0x03,
0xA3, 0x6B,
0xA4, 0x03,
0xA5, 0x7C,
0xA6, 0x03,
0xA7, 0x8F,
0xA9, 0x03,
0xAA, 0xA1,
0xAB, 0x03,
0xAC, 0xB5,
0xAD, 0x03,
0xAE, 0xCB,
0xAF, 0x00,
0xB0, 0x30,
0xB1, 0x00,
0xB2, 0x56,
0xB3, 0x00,
0xB4, 0x84,
0xB5, 0x00,
0xB6, 0xA4,
0xB7, 0x00,
0xB8, 0xBC,
0xB9, 0x00,
0xBA, 0xD1,
0xBB, 0x00,
0xBC, 0xE3,
0xBD, 0x00,
0xBE, 0xF4,
0xBF, 0x01,
0xC0, 0x02,
0xC1, 0x01,
0xC2, 0x33,
0xC3, 0x01,
0xC4, 0x5A,
0xC5, 0x01,
0xC6, 0x94,
0xC7, 0x01,
0xC8, 0xC3,
0xC9, 0x02,
0xCA, 0x09,
0xCB, 0x02,
0xCC, 0x41,
0xCD, 0x02,
0xCE, 0x43,
0xCF, 0x02,
0xD0, 0x73,
0xD1, 0x02,
0xD2, 0xAB,
0xD3, 0x02,
0xD4, 0xCF,
0xD5, 0x02,
0xD6, 0xFE,
0xD7, 0x03,
0xD8, 0x1C,
0xD9, 0x03,
0xDA, 0x43,
0xDB, 0x03,
0xDC, 0x51,
0xDD, 0x03,
0xDE, 0x5D,
0xDF, 0x03,
0xE0, 0x6B,
0xE1, 0x03,
0xE2, 0x7C,
0xE3, 0x03,
0xE4, 0x8F,
0xE5, 0x03,
0xE6, 0xA1,
0xE7, 0x03,
0xE8, 0xB5,
0xE9, 0x03,
0xEA, 0xCB,

//add start
0xFF, 0x20,
0xFB, 0x01,
0x30, 0x00,
0x32, 0x4d,
//add end
#if !(LCM_DSI_CMD_MODE)

0xFF,0x2E,
0xFB,0x01,
0x00,0xAB,
0x03,0x0A,

#endif

0xFF, 0x10,
0x35, 0x00,

#if (LCM_DSI_CMD_MODE)
0x44,0x07,
0x45,0x78,
#endif

0x51, 0xFF,
0x53, 0x2C,
0x55, 0x00,

0xC9, 0x45
};

static void init_lcm_registers(void)
{
	unsigned int data_array[5];
	int i;
	unsigned int data;
	
	data_array[0]=0x00023902;
	for(i = 0; i < (sizeof(reg_init)/(2*sizeof(char))); i++)
	{
		data = 0xff & reg_init[i][1];
		data_array[1]= 0xffff & (data << 8 | 0xff & reg_init[i][0]);
		printk("i = %d,data_array[0] = %x,data_array[1]=%08x\n",i,data_array[0],data_array[1]);	
		dsi_set_cmdq(data_array,2,1);
	}

	data_array[0]=0x00043902;
	data_array[1]=0x0A0A033B;
	dsi_set_cmdq(data_array,2,1);

#if (LCM_DSI_CMD_MODE)
	data_array[0]=0x00023902;
	data_array[1]=0x000010BB;	//0x13:Video mode; 0x10:CMD MODE
	dsi_set_cmdq(data_array,2,1);
	
	data_array[0]=0x000C3902;
	data_array[1]=0x050249C9;	
	data_array[2]=0x67060F00;
	data_array[3]=0xF0102E03;
	dsi_set_cmdq(data_array,4,1);

#else
	data_array[0]=0x00023902;
	data_array[1]=0x000013BB;	//0x13:Video mode; 0x10:CMD MODE
	dsi_set_cmdq(data_array,2,1);
#endif

//***********************************************
//	lcm start display
//***********************************************
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
         params->physical_height = 110000;
        params->physical_width = 62000;
		// enable tearing-free

#if (LCM_DSI_CMD_MODE)
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#else	
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
#endif

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		#if (LCM_DSI_CMD_MODE)	
		params->dsi.intermediat_buffer_num = 4;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
		#else	
		params->dsi.intermediat_buffer_num = 0;
		#endif

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 2;//1;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 13;//2;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;//50;
		params->dsi.horizontal_backporch				= 20;//100;
		params->dsi.horizontal_frontporch				= 40;//80;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable      = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

		// Bit rate calculation
		//1 Every lane speed
		#if 0
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		#endif
#if 0
		
		#if (LCM_DSI_CMD_MODE)
			params->dsi.PLL_CLOCK=330;
		#else
			params->dsi.PLL_CLOCK=340;
		#endif
#else
		params->dsi.PLL_CLOCK=422;
#endif
    //  params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	mt_set_gpio_mode(GPIO195|0x80000000,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO195|0x80000000,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO195|0x80000000,GPIO_OUT_ZERO);
	MDELAY(1);

	cmd=0x00;
	data=0x0c;//x3
	
	mt_set_gpio_mode(GPIO_SGM3804_ENP, GPIO_MODE_00);	//data sheet 136 page ,the first AVDD power on
	mt_set_gpio_dir(GPIO_SGM3804_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENP, GPIO_OUT_ONE);
	
#ifdef BUILD_LK
	ret=sgm3804_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]nt36760----sgm3804----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]nt36760----sgm3804----cmd=%0x--i2c write success----\n",cmd);
#else
	ret=sgm3804_write_bytes(cmd,data);
#endif
	MDELAY(1);

	mt_set_gpio_mode(GPIO_SGM3804_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SGM3804_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENN, GPIO_OUT_ONE);
	cmd=0x01;
	data=0x0c;

#ifdef BUILD_LK
	ret=sgm3804_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]nt36760----sgm3804----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]nt36760----sgm3804----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=sgm3804_write_bytes(cmd,data); 
#endif

	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(5);      

	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(10);

	init_lcm_registers();
}

void lcm_power_off(void)	//temp sloved function null point
{

}
EXPORT_SYMBOL(lcm_power_off);


static void lcm_off(void)
{
	unsigned int data_array[5];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	/*data_array[0] = 0x00FF1500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);*/

	mt_set_gpio_mode(GPIO_SGM3804_ENN, GPIO_MODE_00);	//data sheet 136 page, the first AVDD Power off
	mt_set_gpio_dir(GPIO_SGM3804_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENN, GPIO_OUT_ZERO);
	MDELAY(5);	

	mt_set_gpio_mode(GPIO_SGM3804_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SGM3804_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENP, GPIO_OUT_ZERO);
	MDELAY(5); 

	SET_RESET_PIN(0);
	MDELAY(10);

	mt_set_gpio_mode(GPIO195|0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO195|0x80000000, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO195|0x80000000, GPIO_OUT_ONE);
	MDELAY(5);

}

static void lcm_suspend(void)
{
	/*unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

    data_array[0] = 0x014F1500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);*/
/*	unsigned int data_array[5];
    	
    if(rmi4_tp_wg.wg_enable)
    {
	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
        rmi4_tp_wg.tp_wg_func(rmi4_tp_wg.prmi4_data, true);	
    }
    else
    {*/
	lcm_off();
  //  }
}


static void lcm_resume(void)
{
  /*  if(rmi4_tp_wg.wg_enable)
    {
        rmi4_tp_wg.tp_resume_func(NULL);
	init_lcm_registers();	
    }
    else
    {
	lcm_init();	
    }*/	
    //msleep(500);
    lcm_init();
}


#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

//	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
    
}
#endif

static unsigned int lcm_compare_id(void)
{
#if 1
	unsigned char id=0;
	unsigned char id1=0;
	unsigned char id2=0;
	unsigned char buffer[2];
	unsigned int array[16]; 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xDA, buffer, 1);
	id = buffer[0]; 

	read_reg_v2(0xDB, buffer, 1);
	id1 = buffer[0]; 

	read_reg_v2(0xDC, buffer, 1);
	id2 = buffer[0]; 
#endif


	#if defined(BUILD_LK)
	printf("%s id(%x)  id1(%x) id2(%x)\n", __func__,id,id1,id2);
	#else
	printk("alvin:%s id(%x)  id1(%x) id2(%x)\n", __func__,id,id1,id2);
	#endif
	if(id == 0xe6 && id1 == 0x68 && id2 == 0x81)
		return 1;
	else
		return 0; 
}

LCM_DRIVER nt36760_dsi_vdo_sgmicro_sgm3804_lcm_drv = 
{
    .name			= "nt36760_dsi_vdo_E6_sgm3804",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.esd_check   = lcm_esd_check,
    //.esd_recover   = lcm_esd_recover,

#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

