#include <linux/kernel.h>
#include <linux/string.h>	/* for test cases */
#include <linux/printk.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <asm/atomic.h>
#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include "ddp_reg.h"
#include "ddp_pwm.h"
#include "ddp_path.h"


#define PWM_DEFAULT_DIV_VALUE 0x0

#ifdef CONFIG_MTK_AW2013_LEDS
	#define brightness_a1	25
	#define brightness_b1	332223
	#define brightness_a2	148
	#define brightness_b2	53800
	#define MID_Vale	600
#endif

#define PWM_ERR(fmt, arg...) printk(KERN_ERR "[PWM] " fmt "\n", ##arg)
#define PWM_NOTICE(fmt, arg...) printk(KERN_NOTICE "[PWM] " fmt "\n", ##arg)
#define PWM_MSG(fmt, arg...) printk(KERN_DEBUG "[PWM] " fmt "\n", ##arg)

#define pwm_get_reg_base(id) ((id) == DISP_PWM0 ? DISPSYS_PWM0_BASE : DISPSYS_PWM1_BASE)

#define index_of_pwm(id) ((id == DISP_PWM0) ? 0 : 1)
#define PWM_LOG_BUFFER_SIZE 5


static disp_pwm_id_t g_pwm_main_id = DISP_PWM0;
static atomic_t g_pwm_backlight[2] = { ATOMIC_INIT(-1), ATOMIC_INIT(-1) };
static volatile int g_pwm_max_backlight[2] = { 1023, 1023 };

static ddp_module_notify g_ddp_notify;
static DEFINE_SPINLOCK(g_pwm_log_lock);


typedef struct {
	unsigned int value;
	unsigned long tsec;
	unsigned long tusec;
} PWM_LOG;

enum PWM_LOG_TYPE {
	NOTICE_LOG = 0,
	MSG_LOG,
};

static PWM_LOG g_pwm_log_buffer[PWM_LOG_BUFFER_SIZE + 1];
static int g_pwm_log_index;
static int disp_pwm_config_init(DISP_MODULE_ENUM module, disp_ddp_path_config *pConfig, void *cmdq)
{
	struct cust_mt65xx_led *cust_led_list;
	struct cust_mt65xx_led *cust;
	struct PWM_config *config_data;
	unsigned int pwm_div;
	disp_pwm_id_t id = (module == DISP_MODULE_PWM0 ? DISP_PWM0 : DISP_PWM1);
	unsigned long reg_base = pwm_get_reg_base(id);
	int index = index_of_pwm(id);
	int i;

	pwm_div = PWM_DEFAULT_DIV_VALUE;
	cust_led_list = get_cust_led_list();
	if (cust_led_list) {
		/* WARNING: may overflow if MT65XX_LED_TYPE_LCD not configured properly */
		cust = &cust_led_list[MT65XX_LED_TYPE_LCD];
		if ((strcmp(cust->name, "lcd-backlight") == 0)
		    && (cust->mode == MT65XX_LED_MODE_CUST_BLS_PWM)) {
			config_data = &cust->config_data;
			if (config_data->clock_source >= 0 && config_data->clock_source <= 3) {
				unsigned int regVal = DISP_REG_GET(CLK_CFG_1);
				clkmux_sel(MT_MUX_PWM, config_data->clock_source, "DISP_PWM");
				printk("disp_pwm_init : CLK_CFG_1 0x%x => 0x%x", regVal, DISP_REG_GET(CLK_CFG_1));
			}
			/* Some backlight chip/PMIC(e.g. MT6332) only accept slower clock */
			pwm_div = (config_data->div == 0) ? PWM_DEFAULT_DIV_VALUE : config_data->div;
			pwm_div &= 0x3FF;
			printk("disp_pwm_init : PWM config data (%d,%d)", config_data->clock_source, config_data->div);
		}
	}

	atomic_set(&g_pwm_backlight[index], -1);

	/* We don't enable PWM until we really need */
	DISP_REG_MASK(cmdq, reg_base + DISP_PWM_CON_0_OFF, pwm_div << 16, (0x3ff << 16));

	DISP_REG_MASK(cmdq, reg_base + DISP_PWM_CON_1_OFF, 1023, 0x3ff);	/* 1024 levels */
	/* We don't init the backlight here until AAL/Android give */

	g_pwm_log_index = 0;
	for (i = 0; i < PWM_LOG_BUFFER_SIZE; i += 1) {
		g_pwm_log_buffer[i].tsec = -1;
		g_pwm_log_buffer[i].tusec = -1;
		g_pwm_log_buffer[i].value = -1;
	}

	return 0;
}


static int disp_pwm_config(DISP_MODULE_ENUM module, disp_ddp_path_config *pConfig, void *cmdq)
{
	int ret = 0;

	if (pConfig->dst_dirty)
		ret |= disp_pwm_config_init(module, pConfig, cmdq);

	return ret;
}


static void disp_pwm_trigger_refresh(disp_pwm_id_t id)
{
	DISP_MODULE_ENUM mod = DISP_MODULE_PWM0;

	if (id == DISP_PWM1)
		mod = DISP_MODULE_PWM1;

	if (g_ddp_notify)
		g_ddp_notify(mod, DISP_PATH_EVENT_TRIGGER);
}


/* Set the PWM which acts by default (e.g. ddp_bls_set_backlight) */
void disp_pwm_set_main(disp_pwm_id_t main)
{
	g_pwm_main_id = main;
}


disp_pwm_id_t disp_pwm_get_main(void)
{
	return g_pwm_main_id;
}


int disp_pwm_is_enabled(disp_pwm_id_t id)
{
	unsigned long reg_base = pwm_get_reg_base(id);
	return DISP_REG_GET(reg_base + DISP_PWM_EN_OFF) & 0x1;
}


static void disp_pwm_set_drverIC_en(disp_pwm_id_t id, int enabled)
{
#ifdef GPIO_LCM_LED_EN
	if (id == DISP_PWM0) {
		mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);

		if (enabled)
			mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
		else
			mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
	}
#endif
}


static void disp_pwm_set_enabled(cmdqRecHandle cmdq, disp_pwm_id_t id, int enabled)
{
	unsigned long reg_base = pwm_get_reg_base(id);
	if (enabled) {
		if (!disp_pwm_is_enabled(id)) {
			DISP_REG_MASK(cmdq, reg_base + DISP_PWM_EN_OFF, 0x1, 0x1);
			printk("disp_pwm_set_enabled: PWN_EN = 0x1");

			disp_pwm_set_drverIC_en(id, enabled);
		}
	} else {
		DISP_REG_MASK(cmdq, reg_base + DISP_PWM_EN_OFF, 0x0, 0x1);
		disp_pwm_set_drverIC_en(id, enabled);
	}
}


int disp_bls_set_max_backlight(unsigned int level_1024)
{
	return disp_pwm_set_max_backlight(disp_pwm_get_main(), level_1024);
}


int disp_pwm_set_max_backlight(disp_pwm_id_t id, unsigned int level_1024)
{
	int index;

	if ((DISP_PWM_ALL & id) == 0) {
		PWM_ERR("[ERROR] disp_pwm_set_backlight: invalid PWM ID = 0x%x", id);
		return -EFAULT;
	}

	index = index_of_pwm(id);
	g_pwm_max_backlight[index] = level_1024;

	printk("disp_pwm_set_max_backlight(id = 0x%x, level = %u)", id, level_1024);

	if (level_1024 < atomic_read(&g_pwm_backlight[index]))
		disp_pwm_set_backlight(id, level_1024);

	return 0;
}


int disp_pwm_get_max_backlight(disp_pwm_id_t id)
{
	int index = index_of_pwm(id);
	return g_pwm_max_backlight[index];
}

#define BL_USE_LINEAR_MAPPING

#define MAX_PWM			1023
#define MAX_BRIGHTNESS		255
#ifdef BL_USE_LINEAR_MAPPING
#define MIN_BRIGHTNESS  	5  // [32, 255]
#else
#define MIN_BRIGHTNESS  	192  // [192, 255]
#endif
#define SPACE_BRIGHTNESS  	(MAX_BRIGHTNESS - MIN_BRIGHTNESS + 1)
#define RATIO_BRIGHTNESS	((MAX_PWM + 1) / SPACE_BRIGHTNESS)

#ifdef CONFIG_MTK_LM3533_LEDS
static int enable_flag = -1 ;
#endif

/* For backward compatible */
int disp_bls_set_backlight(int level_1024)
{
#ifdef CONFIG_MTK_LM3533_LEDS
    static int old_value = -1 ;
    int new_value = -1 ;
    int ret; 
    
    if(mt_get_gpio_out(GPIO119|0x80000000)==0)// should not enter this case.
    {
        mt_set_gpio_mode(GPIO119|0x80000000, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO119|0x80000000, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO119|0x80000000, 1);
        printk("get gpio 119 vale(%d)\n",mt_get_gpio_out(GPIO119|0x80000000));
    }

    new_value = MIN_BRIGHTNESS + (level_1024 * SPACE_BRIGHTNESS / (MAX_PWM + 1)) ;
    if (MAX_BRIGHTNESS < new_value) new_value = MAX_BRIGHTNESS ;
    if (0 == new_value && 0 != level_1024) new_value = 1;

    if ((level_1024 > 0) && (1 != enable_flag)) {
        lm3533_backlight_enable() ;
        #ifdef BL_USE_LINEAR_MAPPING
        lm3533_backlight_linear_mapping(1);
        #endif
        lm3533_backlight_current(0x16);  /* 22.6mA */
        lm3533_backlight_pwm(0x01);
        //lm3533_backlight_brightness((u8)new_value) ;
		
        enable_flag = 1 ;
        old_value = -1;
    }

    /* avoid frequently i2c write */
    if (new_value != old_value) {
        old_value = new_value ;
        lm3533_backlight_brightness((u8)new_value) ;
    }

    ret = disp_pwm_set_backlight(disp_pwm_get_main(), level_1024);

    if ((0 == level_1024) && (0 != enable_flag)) {
        lm3533_backlight_disable() ;  
        enable_flag = 0 ;
        old_value = -1;
    }

    return ret;
#endif

#ifdef CONFIG_MTK_AW2013_LEDS
	printk("disp_bls_set_backlight:level_1024 = %d\n",level_1024);
	return disp_pwm_set_backlight(disp_pwm_get_main(), level_1024);
#endif
}


/*
 * If you want to re-map the backlight level from user space to
 * the real level of hardware output, please modify here.
 *
 * Inputs:
 *  id          - DISP_PWM0 / DISP_PWM1
 *  level_1024  - Backlight value in [0, 1023]
 * Returns:
 *  PWM duty in [0, 1023]
 */
static int disp_pwm_level_remap(disp_pwm_id_t id, int level_1024)
{
	return level_1024;
}

#ifdef CONFIG_MTK_AW2013_LEDS
static int disp_pwm_level_remap_Letv(disp_pwm_id_t id, int level_1024)
{
	int new_level_1024 = 0;
	int old_level_1024 = 0;
	if(level_1024 < MID_Vale)
	{
		new_level_1024 = brightness_a1 * level_1024 * level_1024 + level_1024 * brightness_b1 ;
		level_1024 = (int)(new_level_1024 / 1000000);
	}else{
		new_level_1024 = brightness_a2 * level_1024 * level_1024 - level_1024 * brightness_b2 ;
		level_1024 = (int)(new_level_1024 / 100000);
	}
	if(level_1024 > 1 && level_1024 < 10)
		level_1024 = 10;
	if(level_1024 > 900)
		level_1024 = 900;
	return level_1024;
}
#endif

int disp_pwm_set_backlight(disp_pwm_id_t id, int level_1024)
{
	int ret;

	/* Always write registers by CPU */
	ret = disp_pwm_set_backlight_cmdq(id, level_1024, NULL);

	if (ret >= 0)
		disp_pwm_trigger_refresh(id);

	return 0;
}


static volatile int g_pwm_duplicate_count;

static void disp_pwm_log(int level_1024, int log_type)
{
	int i;
	struct timeval pwm_time;
	char buffer[256] = "";
	int print_log;

	do_gettimeofday(&pwm_time);

	spin_lock(&g_pwm_log_lock);

	g_pwm_log_buffer[g_pwm_log_index].value = level_1024;
	g_pwm_log_buffer[g_pwm_log_index].tsec = (unsigned long)pwm_time.tv_sec % 1000;
	g_pwm_log_buffer[g_pwm_log_index].tusec = (unsigned long)pwm_time.tv_usec / 1000;
	g_pwm_log_index += 1;
	print_log = 0;

	if (g_pwm_log_index >= PWM_LOG_BUFFER_SIZE || level_1024 == 0) {
		sprintf(buffer + strlen(buffer), "(latest=%2u): ", g_pwm_log_index);
		for (i = 0; i < g_pwm_log_index; i += 1) {
			sprintf(buffer + strlen(buffer), "%5u(%4lu,%4lu)",
				g_pwm_log_buffer[i].value,
				g_pwm_log_buffer[i].tsec,
				g_pwm_log_buffer[i].tusec);
		}

		g_pwm_log_index = 0;
		print_log = 1;

		for (i = 0; i < PWM_LOG_BUFFER_SIZE; i += 1) {
			g_pwm_log_buffer[i].tsec = -1;
			g_pwm_log_buffer[i].tusec = -1;
			g_pwm_log_buffer[i].value = -1;
		}
	}

	spin_unlock(&g_pwm_log_lock);

	if (print_log == 1) {
		if (log_type == MSG_LOG)
			printk("%s", buffer);
		else
			PWM_NOTICE("%s", buffer);
	}
}

int disp_pwm_set_backlight_cmdq(disp_pwm_id_t id, int level_1024, void *cmdq)
{
	unsigned long reg_base;
	int old_pwm;
	int index;
	int abs_diff;

	if ((DISP_PWM_ALL & id) == 0) {
		PWM_ERR("[ERROR] disp_pwm_set_backlight_cmdq: invalid PWM ID = 0x%x", id);
		return -EFAULT;
	}

	index = index_of_pwm(id);

	old_pwm = atomic_xchg(&g_pwm_backlight[index], level_1024);
	if (old_pwm != level_1024) {
		abs_diff = level_1024 - old_pwm;
		if (abs_diff < 0)
			abs_diff = -abs_diff;

		if (old_pwm == 0 || level_1024 == 0 || abs_diff > 64) {
			/* To be printed in UART log */
			disp_pwm_log(level_1024, MSG_LOG);
			PWM_NOTICE("disp_pwm_set_backlight_cmdq(id = 0x%x, level_1024 = %d), old = %d", id, level_1024,
				   old_pwm);
		} else {
			PWM_MSG("disp_pwm_set_backlight_cmdq(id = 0x%x, level_1024 = %d), old = %d", id, level_1024, old_pwm);
		}

		if (level_1024 > g_pwm_max_backlight[index]) {
			level_1024 = g_pwm_max_backlight[index];
		} else if (level_1024 < 0) {
			level_1024 = 0;
		}

		level_1024 = disp_pwm_level_remap(id, level_1024);
#ifdef CONFIG_MTK_AW2013_LEDS
		level_1024 = disp_pwm_level_remap_Letv(id, level_1024);
#endif
		reg_base = pwm_get_reg_base(id);
		DISP_REG_MASK(cmdq, reg_base + DISP_PWM_CON_1_OFF, level_1024 << 16, 0x1fff << 16);

		if (level_1024 > 0) {
			disp_pwm_set_enabled(cmdq, id, 1);
		} else {
			disp_pwm_set_enabled(cmdq, id, 0);	/* To save power */
		}

		DISP_REG_MASK(cmdq, reg_base + DISP_PWM_COMMIT_OFF, 1, ~0);
		DISP_REG_MASK(cmdq, reg_base + DISP_PWM_COMMIT_OFF, 0, ~0);

		g_pwm_duplicate_count = 0;
	} else {
		g_pwm_duplicate_count = (g_pwm_duplicate_count + 1) & 63;
		if (g_pwm_duplicate_count == 2) {
			PWM_MSG("disp_pwm_set_backlight_cmdq(id = 0x%x, level_1024 = %d), old = %d (dup)",
				 id, level_1024, old_pwm);
		}
	}
#ifdef CONFIG_MTK_AW2013_LEDS
	if(level_1024 == 0)
	{
		mt_set_gpio_mode(GPIO119|0x80000000, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO119|0x80000000, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO119|0x80000000, 0);
	}
	else
	{
		mt_set_gpio_mode(GPIO119|0x80000000, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO119|0x80000000, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO119|0x80000000, 1);
	}
#endif
	return 0;
}


int ddp_pwm_power_on(DISP_MODULE_ENUM module, void *handle)
{
	printk("ddp_pwm_power_on: %d\n", module);

	if (module == DISP_MODULE_PWM0) {
		enable_clock(MT_CG_DISP1_DISP_PWM0_26M, "PWM");
		enable_clock(MT_CG_DISP1_DISP_PWM0_MM, "PWM");
	} else if (module == DISP_MODULE_PWM1) {
		enable_clock(MT_CG_DISP1_DISP_PWM1_26M, "PWM");
		enable_clock(MT_CG_DISP1_DISP_PWM1_MM, "PWM");
	}

	return 0;
}

int ddp_pwm_power_off(DISP_MODULE_ENUM module, void *handle)
{
	printk("ddp_pwm_power_off: %d\n", module);

	if (module == DISP_MODULE_PWM0) {
		atomic_set(&g_pwm_backlight[0], 0);
		disable_clock(MT_CG_DISP1_DISP_PWM0_26M, "PWM");
		disable_clock(MT_CG_DISP1_DISP_PWM0_MM, "PWM");
	} else if (module == DISP_MODULE_PWM1) {
		atomic_set(&g_pwm_backlight[1], 0);
		disable_clock(MT_CG_DISP1_DISP_PWM1_26M, "PWM");
		disable_clock(MT_CG_DISP1_DISP_PWM1_MM, "PWM");
	}

	return 0;
}


static int ddp_pwm_init(DISP_MODULE_ENUM module, void *cmq_handle)
{
	ddp_pwm_power_on(module, cmq_handle);
	return 0;
}

static int ddp_pwm_set_listener(DISP_MODULE_ENUM module, ddp_module_notify notify)
{
	g_ddp_notify = notify;
	return 0;
}



DDP_MODULE_DRIVER ddp_driver_pwm = {
	.init = ddp_pwm_init,
	.config = disp_pwm_config,
	.power_on = ddp_pwm_power_on,
	.power_off = ddp_pwm_power_off,
	.set_listener = ddp_pwm_set_listener,
};



/* ---------------------------------------------------------------------- */
/* Test code */
/* Following is only for PWM functional test, not normal code */
/* Will not be linked into user build. */
/* ---------------------------------------------------------------------- */

static void disp_pwm_test_source(const char *cmd)
{
	unsigned long reg_base = pwm_get_reg_base(DISP_PWM0);
	int sel = (cmd[0] - '0') & 0x3;
	DISP_REG_MASK(NULL, reg_base + DISP_PWM_CON_0_OFF, (sel << 4), (0x3 << 4));
}


static void disp_pwm_test_grad(const char *cmd)
{
	const unsigned long reg_grad = pwm_get_reg_base(DISP_PWM0) + 0x18;

	switch (cmd[0]) {
	case 'H':
		DISP_REG_SET(NULL, reg_grad, (1 << 16) | (1 << 8) | 1);
		disp_pwm_set_backlight(DISP_PWM0, 1023);
		break;

	case 'L':
		DISP_REG_SET(NULL, reg_grad, (1 << 16) | (1 << 8) | 1);
		disp_pwm_set_backlight(DISP_PWM0, 40);
		break;

	default:
		DISP_REG_SET(NULL, reg_grad, 0);
		disp_pwm_set_backlight(DISP_PWM0, 512);
		break;
	}
}


static void disp_pwm_test_div(const char *cmd)
{
	const unsigned long reg_base = pwm_get_reg_base(DISP_PWM0);

	int div = cmd[0] - '0';
	if (div > 5)
		div = 5;

	DISP_REG_MASK(NULL, reg_base + DISP_PWM_CON_0_OFF, div << 16, (0x3ff << 16));
	disp_pwm_set_backlight(DISP_PWM0, 256 + div);	/* to be applied */
}


static void disp_pwm_enable_debug(const char *cmd)
{
	const unsigned long reg_base = pwm_get_reg_base(DISP_PWM0);

	if (cmd[0] == '1')
		DISP_REG_SET(NULL, reg_base + 0x20, 3);
	else
		DISP_REG_SET(NULL, reg_base + 0x20, 0);
}


static void disp_pwm_test_pin_mux(void)
{
	const unsigned long reg_base = pwm_get_reg_base(DISP_PWM1);

	mt_set_gpio_mode(GPIO4, GPIO_MODE_01);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO4, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	mt_set_gpio_mode(GPIO14, GPIO_MODE_03);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO14, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	mt_set_gpio_mode(GPIO127, GPIO_MODE_02);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO127, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	mt_set_gpio_mode(GPIO134, GPIO_MODE_02);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO134, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	mt_set_gpio_mode(GPIO153, GPIO_MODE_05);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO153, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	mt_set_gpio_mode(GPIO186, GPIO_MODE_02);	/* For DVT PIN MUX verification only, not normal path */
	mt_set_gpio_dir(GPIO186, GPIO_DIR_OUT);	/* For DVT PIN MUX verification only, not normal path */

	enable_clock(MT_CG_DISP1_DISP_PWM1_26M, "PWM");
	enable_clock(MT_CG_DISP1_DISP_PWM1_MM, "PWM");

	DISP_REG_MASK(NULL, reg_base + DISP_PWM_CON_1_OFF, 512 << 16, 0x1fff << 16);
	DISP_REG_MASK(NULL, reg_base + DISP_PWM_EN_OFF, 0x1, 0x1);
	DISP_REG_SET(NULL, reg_base + 0x20, 3);
	DISP_REG_MASK(NULL, reg_base + DISP_PWM_COMMIT_OFF, 1, ~0);
	DISP_REG_MASK(NULL, reg_base + DISP_PWM_COMMIT_OFF, 0, ~0);
}


static void disp_pwm_dump(void)
{
	const unsigned long reg_base = pwm_get_reg_base(DISP_PWM0);
	int offset;

	PWM_NOTICE("[DUMP] Base = 0x%lx", reg_base);
	for (offset = 0; offset <= 0x28; offset += 4) {
		unsigned int val = DISP_REG_GET(reg_base + offset);
		PWM_NOTICE("[DUMP] [+0x%02x] = 0x%08x", offset, val);
	}
}


void disp_pwm_test(const char *cmd, char *debug_output)
{
	debug_output[0] = '\0';
	PWM_NOTICE("disp_pwm_test(%s)", cmd);

	if (strncmp(cmd, "src:", 4) == 0)
		disp_pwm_test_source(cmd + 4);
	else if (strncmp(cmd, "div:", 4) == 0)
		disp_pwm_test_div(cmd + 4);
	else if (strncmp(cmd, "grad:", 5) == 0)
		disp_pwm_test_grad(cmd + 5);
	else if (strncmp(cmd, "dbg:", 4) == 0)
		disp_pwm_enable_debug(cmd + 4);
	else if (strncmp(cmd, "dump", 4) == 0)
		disp_pwm_dump();
	else if (strncmp(cmd, "pinmux", 6) == 0)
		disp_pwm_test_pin_mux();
}
