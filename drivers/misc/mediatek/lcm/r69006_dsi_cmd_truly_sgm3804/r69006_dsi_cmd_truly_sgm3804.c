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

#define LCM_ID_r69006 (0x9006)

//#define GPIO_SGM3804_ENN   GPIO_LCD_BIAS_ENN_PIN
//#define GPIO_SGM3804_ENP   GPIO_LCD_BIAS_ENP_PIN
#define GPIO_SGM3804_ENN   (GPIO131|0x80000000)
#define GPIO_SGM3804_ENP   (GPIO9  |0x80000000)
#define  LCM_DSI_CMD_MODE	1

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

#endif


#ifdef BUILD_LK

#define I2C_I2C_LCD_BIAS_CHANNEL  2
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

	extern int sgm3804_write_bytes(unsigned char addr, unsigned char value);

#endif


static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	int i;
	unsigned int data;
	
	SET_RESET_PIN(1);
#if 0//sang
	MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(10);
        SET_RESET_PIN(1);
#endif
    	MDELAY(10);

	data_array[0] = 0x00b02300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00062902;
#if (LCM_DSI_CMD_MODE)
	data_array[1] = 0x001004b3;//cmd mode
#else
	data_array[1] = 0x001005b3;//video mode
#endif
	data_array[2] = 0x00000000;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x00000cb4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00042902;
	data_array[1] = 0x00d33bb6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00c02300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x98361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04cc2300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00202902;
	data_array[1] = 0x100084c1;
	data_array[2] = 0xfff18bef;
	data_array[3] = 0xc59cdfff;
	data_array[4] = 0xad8d739a;
	data_array[5] = 0xfffffe63;
	data_array[6] = 0x0001f8cb;
	data_array[7] = 0xc20040aa;
	data_array[8] = 0x01000801;
	dsi_set_cmdq(data_array, 9, 1);
	data_array[0] = 0x000a2902;
	data_array[1] = 0x1ffe0dcb;
	data_array[2] = 0x0000002c;
	data_array[3] = 0x00000000;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x000b2902;
	data_array[1] = 0x80f701c2;
	data_array[2] = 0x60006304;
	data_array[3] = 0x00300100;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x00072902;
	data_array[1] = 0x000155c3;
	data_array[2] = 0x00000001;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00122902;
	data_array[1] = 0x000070c4;
	data_array[2] = 0x00000000;
	data_array[3] = 0x01020000;
	data_array[4] = 0x00010500;
	data_array[5] = 0x00000000;
	dsi_set_cmdq(data_array, 6, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x4a0759c6;
	data_array[2] = 0x0e014a07;
	data_array[3] = 0x02010201;
	data_array[4] = 0x00071509;
	dsi_set_cmdq(data_array, 5, 1);
	data_array[0] = 0x001f2902;
data_array[1] = 0x1e1100c7;
data_array[2] = 0x4e45362a;
data_array[3] = 0x50463e5a;
data_array[4] = 0x7a706058;
data_array[5] = 0x2a1e1100;
data_array[6] = 0x5a4e4536;
data_array[7] = 0x5850463e;
data_array[8] = 0x007a7060;
	dsi_set_cmdq(data_array, 9, 1);
	data_array[0] = 0x00142902;
data_array[1] = 0xfe0000c8;
data_array[2] = 0x00e70801;
data_array[3] = 0x0302fd00;
data_array[4] = 0xfc0000a8;
data_array[5] = 0x00c9e9e7;
	dsi_set_cmdq(data_array, 6, 1);
	data_array[0] = 0x00092902;
	data_array[1] = 0x1f681fc9;
	data_array[2] = 0xc44c4c68;
	data_array[3] = 0x00000011;
	dsi_set_cmdq(data_array, 4, 1);
// start demon
	data_array[0] = 0x00022902;
	data_array[1] = 0x000004be;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00112902;
	data_array[1] = 0x001040cf;
	data_array[2] = 0x32000000;
	data_array[3] = 0x32000000;
	data_array[4] = 0x00000000;
	data_array[5] = 0x00000000;
	dsi_set_cmdq(data_array, 6, 1);

	data_array[0] = 0x00062902;
	data_array[1] = 0x3f0000de;
	data_array[2] = 0x000010ff;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000000e9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000000f2;
	dsi_set_cmdq(data_array, 2, 1);
// end demon
	data_array[0] = 0x00112902;
	data_array[1] = 0x910133d0;
	data_array[2] = 0x1919d90b;
	data_array[3] = 0x19000000;
	data_array[4] = 0x00000099;
	data_array[5] = 0x00000000;
	dsi_set_cmdq(data_array, 6, 1);
	data_array[0] = 0x001d2902;
	data_array[1] = 0xbb3b1bd3;
	data_array[2] = 0x3333a5ad;
	data_array[3] = 0xad800033;
data_array[4] = 0x335b5ba8;
	data_array[5] = 0xf2f73333;
	data_array[6] = 0xff7c7d1f;
	data_array[7] = 0xff00990f;
	data_array[8] = 0x000000ff;
	dsi_set_cmdq(data_array, 9, 1);
	data_array[0] = 0x00042902;
	data_array[1] = 0x033357d4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x000c2902;
	data_array[1] = 0x000066d5;
data_array[2] = 0x24012401;
data_array[3] = 0x34003400;
	dsi_set_cmdq(data_array, 4, 1);
	data_array[0] = 0x01d62300;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00351500;//set TE
	dsi_set_cmdq(data_array, 1, 1);
	

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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;

//		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

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
		
		params->dsi.cont_clock = 0;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=720*3;	

		/*
		params->dsi.vertical_sync_active				= 2;//1;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 13;//2;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;//50;
		params->dsi.horizontal_backporch				= 20;//100;
		params->dsi.horizontal_frontporch				= 40;//80;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		*/
		params->dsi.vertical_sync_active				= 15;//1;
		params->dsi.vertical_backporch					= 9;
		params->dsi.vertical_frontporch					= 3;//2;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 20;//20//////===50
		params->dsi.horizontal_backporch				= 82;//80//////===120
		params->dsi.horizontal_frontporch				= 100;//100////====120
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable      = 0;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

		//params->dsi.clk_lp_per_line_enable = true;

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
		params->dsi.PLL_CLOCK=460;
#endif
    //  params->dsi.ssc_disable = 1;
}

static void lcm_init(void)
{
	
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;

	mt_set_gpio_mode(GPIO195|0X80000000,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO195|0X800000000,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO195|0X800000000,GPIO_OUT_ZERO);
	MDELAY(1);
   
	cmd=0x00;
	data=0x0f;//x3
	
	mt_set_gpio_mode(GPIO_SGM3804_ENP, GPIO_MODE_00);	//data sheet 136 page ,the first AVDD power on
	mt_set_gpio_dir(GPIO_SGM3804_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENP, GPIO_OUT_ONE);
	
#ifdef BUILD_LK
	ret=sgm3804_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]r69006----sgm3804----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]r69006----sgm3804----cmd=%0x--i2c write success----\n",cmd);
#else
	ret=sgm3804_write_bytes(cmd,data);
#endif
	MDELAY(1);

	mt_set_gpio_mode(GPIO_SGM3804_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SGM3804_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENN, GPIO_OUT_ONE);
	cmd=0x01;
	data=0x0f;

#ifdef BUILD_LK
	ret=sgm3804_write_byte(cmd,data);
	if(ret)    	
		dprintf(0, "[LK]r69006----sgm3804----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]r69006----sgm3804----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=sgm3804_write_bytes(cmd,data); 
#endif

	MDELAY(5);

	
	init_lcm_registers();

#if 1  //tp reset pin low
	mt_set_gpio_mode(GPIO_CTP_RST_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ONE);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
#endif
}

static void lcm_off(void)
{
	SET_RESET_PIN(0);
	MDELAY(7);
	mt_set_gpio_mode(GPIO_SGM3804_ENN, GPIO_MODE_00);	//data sheet 136 page, the first AVDD Power off
	mt_set_gpio_dir(GPIO_SGM3804_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENN, GPIO_OUT_ZERO);
	MDELAY(10);	

	mt_set_gpio_mode(GPIO_SGM3804_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SGM3804_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENP, GPIO_OUT_ZERO);
	MDELAY(10); 

#if 1
	mt_set_gpio_mode(GPIO195|0x80000000, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO195|0x80000000, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO195|0x80000000, GPIO_OUT_ONE);
	MDELAY(5); 
#endif

}
extern int truly_flag;
static void lcm_suspend(void)
{
	unsigned int data_array[5];
    printk("truly lcm_suspend truly_flag = %d\n",truly_flag);
    if(truly_flag)
    {
        data_array[0] = 0x00100500; // Sleep In
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

        data_array[0]=0x00280500; // Display Off
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20); 
    }
    else
    {
#if 1  //tp reset pin low
	mt_set_gpio_mode(GPIO_CTP_RST_PIN,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN,GPIO_OUT_ZERO);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
#endif

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00052902;
	data_array[1] = 0xbb3b13d3;
	data_array[2] = 0x000000a5;
	dsi_set_cmdq(data_array,3,1);

	MDELAY(20); 

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);
/*
	mt_set_gpio_mode(GPIO_SGM3804_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SGM3804_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENN, 0);
	MDELAY(15);

	mt_set_gpio_mode(GPIO_SGM3804_ENP, GPIO_MODE_00);	//data sheet 136 page ,the first AVDD power on
	mt_set_gpio_dir(GPIO_SGM3804_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SGM3804_ENP, 0);
	MDELAY(30);
*/	

	lcm_off();
    }
}


static void lcm_resume(void)
{
/*	unsigned int data_array[5];

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

	//init_lcm_registers();
        //msleep(500);
    printk("truly lcm_resume truly_flag = %d\n",truly_flag);
    if(truly_flag)
    {
        init_lcm_registers();
    }
    else
    {
       lcm_init();
    }
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

#define Truly 1234
#define Auo   2345
#define Sharp 3456

static unsigned int lcm_compare_id(void)
{
unsigned char buffer[5];
	unsigned int array[16];  
	int i;
	unsigned int lcd_id = 0;
	SET_RESET_PIN(1);
	MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(10);
        SET_RESET_PIN(1);
    	MDELAY(50);
	array[0] = 0x00b02300;
	dsi_set_cmdq(array, 1, 1);
	array[0] = 0x00053700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
  
	read_reg_v2(0xBF, buffer, 5);
	MDELAY(10);
	lcd_id = (buffer[2] << 8 )| buffer[3];

#ifdef BUILD_LK
    dprintf(0, "%s, LK r63419 debug: r63419 id = 0x%08x\n", __func__, lcd_id);
#else
    printk("alvin:%s, kernel r63419 horse debug: r63419 id = 0x%08x\n", __func__, lcd_id);
#endif
    if(lcd_id == LCM_ID_r69006)
        return Truly;
    else
        return 0;
}

LCM_DRIVER r69006_dsi_cmd_truly_sgm3804_lcm_drv = 
{
    	.name   	= "r69006_dsi_cmd_37_sgm3804",
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

