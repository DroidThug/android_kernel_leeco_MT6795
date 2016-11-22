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
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_NT35590 (0x90)
//#define GPIO_65132_ENN   GPIO_LCD_BIAS_ENN_PIN
//#define GPIO_65132_ENP   GPIO_LCD_BIAS_ENP_PIN
#define GPIO_65132_ENN (GPIO131|0x80000000)
#define GPIO_65132_ENP (GPIO9  |0x80000000)
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
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


//#define GPIO_65132_ENN   GPIO_LCD_BIAS_ENN_PIN
//#define GPIO_65132_ENP   GPIO_LCD_BIAS_ENP_PIN

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

#define TPS_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E

/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct tps65132_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "tps65132_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
  printk( "tps65132_remove\n");
  tps65132_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("tps65132 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

   printk( "tps65132_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
   printk( "tps65132_iic_init2\n");
   i2c_add_driver(&tps65132_iic_driver);
   printk( "tps65132_iic_init success\n");	
   return 0;
}

static void __exit tps65132_iic_exit(void)
{
  printk( "tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 

#endif



#ifdef BUILD_LK

#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;

static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
	
//	#define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data)
	//#define TPS65132_read_byte(add)  mt8193_i2c_read(add)
  

#endif

#define   LCM_DSI_CMD_MODE							0


static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00FF1500;
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10);

        data_array[0] = 0x014F1500;   
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(5);  

        SET_RESET_PIN(0);
        MDELAY(5);
        SET_RESET_PIN(1);
        MDELAY(15);

	data_array[0] = 0x00FF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01FB1500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(30);

	data_array[0] = 0x00FF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08D31500;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0ED41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(40);

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
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

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

		// Video mode setting		
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

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
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable      = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		params->dsi.vertical_frontporch_for_low_power   = 0;
		// Bit rate calculation
		//1 Every lane speed
		#if 0
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =0x12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		#else
              params->dsi.PLL_CLOCK=422;
		#endif
        
        params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x12;

	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]nt35532----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]nt35532----tps6132----cmd=%0x--i2c write success----\n",cmd);
#else
	ret=tps65132_write_bytes(cmd,data);
#endif

	MDELAY(5);

	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	cmd=0x01;
	data=0x10;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]nt35532----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]nt35532----tps6132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps65132_write_bytes(cmd,data); 
#endif

	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(50);      

	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(15);

	init_lcm_registers();
}

void lcm_power_off(void);


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
    lcm_power_off();
}


static void lcm_resume(void)
{
	/*unsigned int data_array[16];

	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(20);

	data_array[0] = 0x00FF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08D31500;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0ED41500;
	dsi_set_cmdq(data_array, 1, 1);
    
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(40);*/
    lcm_init();
}

void lcm_power_off(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	/*data_array[0] = 0x00FF1500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);*/

	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
	MDELAY(10); 
	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);

	MDELAY(50); 

	//printk("justin lcm_power_off test\n");

	SET_RESET_PIN(0);

	/*mt_set_gpio_mode((GPIO85| 0x80000000), GPIO_MODE_00);
	mt_set_gpio_dir((GPIO85| 0x80000000), GPIO_DIR_OUT);
	mt_set_gpio_out((GPIO85| 0x80000000), GPIO_OUT_ZERO);*/
	MDELAY(5); 
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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
    
}
#endif

#define Truly 1234
#define Auo   2345
#define Sharp 3456

static unsigned int lcm_compare_id(void)
{
/*	unsigned int id=0;
	unsigned int id1=0;
	unsigned int id2=0;
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

	#if defined(BUILD_LK)
	printf("%s id(%x)  id1(%x) id2(%x)\n", __func__,id,id1,id2);
	#endif
*/
	return Sharp;//temp solution for two lcm 
}

LCM_DRIVER nt35532_fhd_dsi_video_sharp_tps65132_lcm_drv = 
{
    .name			= "nt35532_fhd_dsi_video_sharp_tps65132",
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
