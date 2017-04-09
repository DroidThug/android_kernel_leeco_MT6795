//********************************************************************************
//
//		<< LC898122 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898122 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition						2009.07.30 Y.Tashita
//********************************************************************************

#ifdef	OISINI_OFILM 
	#define	OISINI_OFILM__
#else
	#define	OISINI_OFILM__		extern
#endif







#ifdef	OISCMD_OFILM
	#define	OISCMD_OFILM__
#else
	#define	OISCMD_OFILM__		extern
#endif

//#define	INIT_FAST_OFILM

// Define According To Usage

/****************************** Define説明 ******************************/
/*	USE_3WIRE_DGYRO		Digital Gyro I/F 3線Mode使用					*/
/*	USE_INVENSENSE_OFILM		Invensense Digital Gyro使用						*/
/*		USE_IDG2020		Inv IDG-2020使用								*/
/*	STANDBY_MODE_OFILM		Standby制御使用(未確認)							*/
/*	GAIN_CONT_OFILM			:Gain control機能使用							*/
/*		(disable)		DSC			:三脚Mode使用						*/
/*	HALLADJ_HW_OFILM			Hall Calibration LSI機能使用					*/
/************************************************************************/

/**************** Select Gyro Sensor **************/
//#define 	USE_3WIRE_DGYRO    //for D-Gyro SPI interface

#define		USE_INVENSENSE_OFILM		// INVENSENSE
#ifdef USE_INVENSENSE_OFILM
//			#define		FS_SEL_OFILM		0		/* ±262LSB/°/s  */
//			#define		FS_SEL_OFILM		1		/* ±131LSB/°/s  */
//			#define		FS_SEL_OFILM		2		/* ±65.5LSB/°/s  */
			#define		FS_SEL_OFILM		3		/* ±32.8LSB/°/s  */

//			#define		GYROSTBY			/* Sleep+STBY */
#endif

/**************** Model name *****************/
#define MDL_VER_OFILM         0x02

/**************** FW version *****************/
#define	FW_VER_OFILM			0x01
 
/**************** Select Mode **************/
#define		STANDBY_MODE_OFILM		// STANDBY Mode
#define		GAIN_CONT_OFILM			// Gain Control Mode
//#define		HALLADJ_HW_OFILM			// H/W Hall adjustment 
//#define		PWM_BREAK			// PWM mode select (disable standby)

#define		DEF_SET_OFILM				// default value re-setting
//#define		USE_EXTCLK_ALL_OFILM		// USE Ext clk for ALL
//#define		USE_EXTCLK_PWM		// USE Ext clk for PWM
//#define		USE_VH_SYNC			// USE V/H Sync for PWM
//#define		PWM_CAREER_TEST		// PWM_CAREER_TEST
#define		NEUTRAL_CENTER_OFILM		// Upper Position Current 0mA Measurement
#define		H1COEF_CHANGER_OFILM			/* H1 coef lvl chage */
#define		MONITOR_OFF_OFILM_OFILM			// default Monitor output
#define		MODULE_CALIBRATION_OFILM		// for module maker   use float
//#define		AF_PWMMODE			// AF Driver PWM mode
#define		CORRECT_1DEG_OFILM			// Correct 1deg   disable 0.5deg
#define		ACCEPTANCE_OFILM					// Examination of Acceptance 



// Command Status
#define		EXE_END_OFILM		0x02		// Execute End (Adjust OK)
#define		EXE_HXADJ_OFILM	0x06		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ_OFILM	0x0A		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ_OFILM	0x12		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ_OFILM	0x22		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ_OFILM	0x42		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ_OFILM	0x82		// Adjust NG : Y Gyro NG (offset)
#define		EXE_OCADJ_OFILM	0x402		// Adjust NG : OSC Clock NG
#define		EXE_ERR_OFILM		0x99		// Execute Error End

#ifdef	ACCEPTANCE_OFILM
 // Hall Examination of Acceptance
 #define		EXE_HXMVER_OFILM	0x06		// X Err
 #define		EXE_HYMVER_OFILM	0x0A		// Y Err
 
 // Gyro Examination of Acceptance
 #define		EXE_GXABOVE_OFILM	0x06		// X Above
 #define		EXE_GXBELOW_OFILM	0x0A		// X Below
 #define		EXE_GYABOVE_OFILM	0x12		// Y Above
 #define		EXE_GYBELOW_OFILM	0x22		// Y Below
#endif	//ACCEPTANCE_OFILM

// Common Define
#define	SUCCESS_OFILM			0x00		// Success
#define	FAILURE_OFILM			0x01		// Failure

#ifndef ON_OFILM
 #define	ON_OFILM				0x01		// ON_OFILM
 #define	OFF_OFILM				0x00		// OFF_OFILM
#endif
 #define	SPC_OFILM				0x02		// Special Mode

#define	X_DIR_OFILM			0x00		// X Direction
#define	Y_DIR_OFILM			0x01		// Y Direction
#define	X2_DIR_OFILM			0x10		// X Direction
#define	Y2_DIR_OFILM			0x11		// Y Direction

#define	NOP_TIME_OFILM		0.00004166F

#ifdef STANDBY_MODE_OFILM
 // Standby mode
 #define		STB1_ON_OFILM		0x00		// Standby1 ON_OFILM
 #define		STB1_OFF_OFILM	0x01		// Standby1 OFF_OFILM
 #define		STB2_ON_OFILM		0x02		// Standby2 ON_OFILM
 #define		STB2_OFF_OFILM	0x03		// Standby2 OFF_OFILM
 #define		STB3_ON_OFILM		0x04		// Standby3 ON_OFILM
 #define		STB3_OFF_OFILM	0x05		// Standby3 OFF_OFILM
 #define		STB4_ON_OFILM		0x06		// Standby4 ON_OFILM			/* for Digital Gyro Read */
 #define		STB4_OFF_OFILM	0x07		// Standby4 OFF_OFILM
 #define		STB2_OISON_OFILM	0x08		// Standby2 ON_OFILM (only OIS)
 #define		STB2_OISOFF_OFILM	0x09		// Standby2 OFF_OFILM(only OIS)
 #define		STB2_AFON_OFILM	0x0A		// Standby2 ON_OFILM (only AF)
 #define		STB2_AFOFF_OFILM	0x0B		// Standby2 OFF_OFILM(only AF)
#endif


// OIS Adjust Parameter
 #define		DAHLXO_INI_OFILM		0x0000
 #define		DAHLXB_INI_OFILM		0xE000
 #define		DAHLYO_INI_OFILM		0x0000
 #define		DAHLYB_INI_OFILM		0xE000
 #define		SXGAIN_INI_OFILM		0x3000
 #define		SYGAIN_INI_OFILM		0x3000
 #define		HXOFF0Z_INI_OFILM		0x0000
 #define		HYOFF1Z_INI_OFILM		0x0000

#if 1	// Avoid Hall Noise   
 #define        BIAS_CUR_OIS_OFILM    0x44         //3.0mA/3.0mA  
 #define        AMP_GAIN_X_OFILM      0x03         //x75
 #define        AMP_GAIN_Y_OFILM      0x03         //x75  
#else  
 #define		BIAS_CUR_OIS_OFILM	0x33		//2.0mA/2.0mA
 #define		AMP_GAIN_X_OFILM		0x05		//x150
 #define		AMP_GAIN_Y_OFILM		0x05		//x150
#endif

/* OSC Init */
 #define		OSC_INI_OFILM			0x2E		/* VDD=2.8V */

/* AF Open para */
 #define		RWEXD1_L_AF_OFILM		0x7FFF		//
 #define		RWEXD2_L_AF_OFILM		0x146C		// abe 2014.11.12
 #define		RWEXD3_L_AF_OFILM		0x6F50		// abe 2014.11.12
 #define		FSTCTIME_AF_OFILM		0xD1		// abe 2014.11.12
 
 #define		FSTMODE_AF_OFILM		0x00		// abe 2014.06.03
 
 /* (0.3750114X^3+0.5937681X)*(0.3750114X^3+0.5937681X) 6.5ohm*/
// #define		A3_IEXP3_OFILM		0x3EC0017F
// #define		A1_IEXP1_OFILM		0x3F180130
 /* (0.425X^3+0.55X)*(0.425X^3+0.55X) 10.2ohm*/
 #define		A3_IEXP3_OFILM		0x3ED9999A
 #define		A1_IEXP1_OFILM		0x3F0CCCCD 
 
/* AF adjust parameter */
#define		DAHLZB_INI_OFILM		0x8001		// abe 2014.06.03
#define		DAHLZO_INI_OFILM		0x0000
#define		BIAS_CUR_AF_OFILM		0x00		//0.25mA
#define		AMP_GAIN_AF_OFILM		0x00		//x6

// Digital Gyro offset Initial value 
#define		DGYRO_OFST_XH_OFILM	0x00
#define		DGYRO_OFST_XL_OFILM	0x00
#define		DGYRO_OFST_YH_OFILM	0x00
#define		DGYRO_OFST_YL_OFILM	0x00

#define		SXGAIN_LOP_OFILM		0x3000
#define		SYGAIN_LOP_OFILM		0x3000

#define		TCODEH_ADJ_OFILM		0x0000

#define		GYRLMT1H_OFILM		0x3DCCCCCD		//0.1F

#ifdef	CORRECT_1DEG_OFILM
// #define		GYRLMT3_S1_OFILM		0x3F19999A		//0.60F
 //#define		GYRLMT3_S2_OFILM		0x3F19999A		//0.60F
//#define 	   GYRLMT3_S1_OFILM 	   0x3F000000	   //0.50f
//#define 	   GYRLMT3_S2_OFILM 	   0x3F000000	   //0.50f
 #define		GYRLMT3_S1_OFILM		0x3ECCCCCD		//0.40F
 #define		GYRLMT3_S2_OFILM		0x3ECCCCCD		//0.40F


 #define		GYRLMT4_S1_OFILM		0x40400000		//3.0F
 #define		GYRLMT4_S2_OFILM		0x40400000		//3.0F

// #define		GYRA12_HGH_OFILM		0x40000000		/* 2.00F */
  #define		GYRA12_HGH_OFILM		0x402CCCCD		/* 2.70F */		// modified by abe 140814
 #define		GYRA12_MID_OFILM		0x3F800000		/* 1.0F */
 #define		GYRA34_HGH_OFILM		0x3F000000		/* 0.5F */
 #define		GYRA34_MID_OFILM		0x3DCCCCCD		/* 0.1F */

 #define		GYRB12_HGH_OFILM		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID_OFILM		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_HGH_OFILM		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_MID_OFILM		0x3C23D70A		/* 0.001F */

#else
 #define		GYRLMT3_S1_OFILM		0x3ECCCCCD		//0.40F
 #define		GYRLMT3_S2_OFILM		0x3ECCCCCD		//0.40F

 #define		GYRLMT4_S1_OFILM		0x40000000		//2.0F
 #define		GYRLMT4_S2_OFILM		0x40000000		//2.0F

 #define		GYRA12_HGH_OFILM		0x3FC00000		/* 1.50F */
 #define		GYRA12_MID_OFILM		0x3F800000		/* 1.0F */
 #define		GYRA34_HGH_OFILM		0x3F000000		/* 0.5F */
 #define		GYRA34_MID_OFILM		0x3DCCCCCD		/* 0.1F */

 #define		GYRB12_HGH_OFILM		0x3E4CCCCD		/* 0.20F */
 #define		GYRB12_MID_OFILM		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_HGH_OFILM		0x3CA3D70A		/* 0.02F */
 #define		GYRB34_MID_OFILM		0x3C23D70A		/* 0.001F */

#endif


//#define		OPTCEN_X_OFILM		0x0000
//#define		OPTCEN_Y_OFILM		0x0000

#ifdef USE_INVENSENSE_OFILM
  #define		SXQ_INI_OFILM			0x3F800000
  #define		SYQ_INI_OFILM		0xBF800000	// 2014.01.20 abe

  #define		GXGAIN_INI_OFILM		0x3F19999A  
  #define		GYGAIN_INI_OFILM		0x3F19999A  

  #define		GYROX_INI_OFILM		0x45
  #define		GYROY_INI_OFILM		0x43
    
  #define		GXHY_GYHX_OFILM		0
#endif


/* Optical Center & Gyro Gain for Mode */
 #define	VAL_SET_OFILM				0x00		// Setting mode
 #define	VAL_FIX_OFILM				0x01		// Fix Set value
 #define	VAL_SPC_OFILM				0x02		// Special mode


struct STFILREG_OFILM {
	unsigned short	UsRegAdd_OFILM ;
	unsigned char	UcRegDat_OFILM ;
} ;													// Register Data Table

struct STFILRAM_OFILM {
	unsigned short	UsRamAdd_OFILM ;
	unsigned long	UlRamDat_OFILM ;
} ;													// Filter Coefficient Table

struct STCMDTBL_OFILM
{
	unsigned short Cmd_OFILM ;
	unsigned int UiCmdStf_OFILM ;
	void ( *UcCmdPtr_OFILM )( void ) ;
} ;

/*** caution [little-endian] ***/

// Word Data Union
union	WRDVAL_OFILM{
	unsigned short	UsWrdVal_OFILM ;
	unsigned char	UcWrkVal_OFILM[ 2 ] ;
	struct {
		unsigned char	UcLowVal_OFILM ;
		unsigned char	UcHigVal_OFILM ;
	} StWrdVal_OFILM ;
} ;

typedef union WRDVAL_OFILM	UnWrdVal_OFILM ;

union	DWDVAL_OFILM {
	unsigned long	UlDwdVal_OFILM ;
	unsigned short	UsDwdVal_OFILM[ 2 ] ;
	struct {
		unsigned short	UsLowVal_OFILM ;
		unsigned short	UsHigVal_OFILM ;
	} StDwdVal_OFILM ;
	struct {
		unsigned char	UcRamVa0_OFILM ;
		unsigned char	UcRamVa1_OFILM ;
		unsigned char	UcRamVa2_OFILM ;
		unsigned char	UcRamVa3_OFILM ;
	} StCdwVal_OFILM ;
} ;

typedef union DWDVAL_OFILM	UnDwdVal_OFILM;

// Float Data Union
union	FLTVAL_OFILM {
	float			SfFltVal_OFILM ;
	unsigned long	UlLngVal_OFILM ;
	unsigned short	UsDwdVal_OFILM[ 2 ] ;
	struct {
		unsigned short	UsLowVal_OFILM ;
		unsigned short	UsHigVal_OFILM ;
	} StFltVal_OFILM ;
} ;

typedef union FLTVAL_OFILM	UnFltVal_OFILM ;


typedef struct STADJPAR_OFILM {
	struct {
		unsigned char	UcAdjPhs_OFILM ;				// Hall Adjust Phase

		unsigned short	UsHlxCna_OFILM ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlxMax_OFILM ;				// Hall Max Value
		unsigned short	UsHlxMxa_OFILM ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlxMin_OFILM ;				// Hall Min Value
		unsigned short	UsHlxMna_OFILM ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlxGan_OFILM ;				// Hall Gain Value
		unsigned short	UsHlxOff_OFILM ;				// Hall Offset Value
		unsigned short	UsAdxOff_OFILM ;				// Hall A/D Offset Value
		unsigned short	UsHlxCen_OFILM ;				// Hall Center Value

		unsigned short	UsHlyCna_OFILM ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlyMax_OFILM ;				// Hall Max Value
		unsigned short	UsHlyMxa_OFILM ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlyMin_OFILM ;				// Hall Min Value
		unsigned short	UsHlyMna_OFILM ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlyGan_OFILM ;				// Hall Gain Value
		unsigned short	UsHlyOff_OFILM ;				// Hall Offset Value
		unsigned short	UsAdyOff_OFILM ;				// Hall A/D Offset Value
		unsigned short	UsHlyCen_OFILM ;				// Hall Center Value
	} StHalAdj_OFILM ;

	struct {
		unsigned short	UsLxgVal_OFILM ;				// Loop Gain X
		unsigned short	UsLygVal_OFILM ;				// Loop Gain Y
		unsigned short	UsLxgSts_OFILM ;				// Loop Gain X Status
		unsigned short	UsLygSts_OFILM ;				// Loop Gain Y Status
	} StLopGan_OFILM ;

	struct {
		unsigned short	UsGxoVal_OFILM ;				// Gyro A/D Offset X
		unsigned short	UsGyoVal_OFILM ;				// Gyro A/D Offset Y
		unsigned short	UsGxoSts_OFILM ;				// Gyro Offset X Status
		unsigned short	UsGyoSts_OFILM ;				// Gyro Offset Y Status
	} StGvcOff_OFILM ;
	
	unsigned char		UcOscVal_OFILM ;				// OSC value

} stAdjPar_OFILM ;

OISCMD_OFILM__	stAdjPar_OFILM	StAdjPar_OFILM ;				// Execute Command Parameter

OISCMD_OFILM__	unsigned char	UcOscAdjFlg_OFILM ;		// For Measure trigger
  #define	MEASSTR_OFILM		0x01
  #define	MEASCNT_OFILM		0x08
  #define	MEASFIX_OFILM		0x80

OISINI_OFILM__	unsigned short	UsCntXof_OFILM ;				/* OPTICAL Center Xvalue */
OISINI_OFILM__	unsigned short	UsCntYof_OFILM ;				/* OPTICAL Center Yvalue */

OISINI_OFILM__	unsigned char	UcPwmMod_OFILM ;				/* PWM MODE */
#define		PWMMOD_CVL_OFILM	0x00		// CVL PWM MODE
#define		PWMMOD_PWM_OFILM	0x01		// PWM MODE

#define		INIT_PWMMODE_OFILM	PWMMOD_CVL_OFILM		// initial output mode

OISINI_OFILM__	unsigned char	UcCvrCod_OFILM ;				/* CverCode */
 #define	CVER122_OFILM		0x93		 // LC898122
 #define	CVER122A_OFILM	0xA1		 // LC898122A


// Prottype Declation
OISINI_OFILM__ void	IniSet_OFILM( void ) ;													// Initial Top Function
OISINI_OFILM__ void	IniSetAf_OFILM( void ) ;													// Initial Top Function

OISINI_OFILM__ void	ClrGyr_OFILM( unsigned short, unsigned char ); 							   // Clear Gyro RAM
	#define CLR_FRAM0_OFILM		 	0x01
	#define CLR_FRAM1_OFILM 			0x02
	#define CLR_ALL_RAM_OFILM 		0x03
OISINI_OFILM__ void	BsyWit_OFILM( unsigned short, unsigned char ) ;				// Busy Wait Function
OISINI_OFILM__ void	WitTim_Ofilm( unsigned short ) ;											// Wait
OISINI_OFILM__ void	MemClr_OFILM( unsigned char *, unsigned short ) ;							// Memory Clear Function
OISINI_OFILM__ void	GyOutSignal_OFILM( void ) ;									// Slect Gyro Output signal Function
OISINI_OFILM__ void	GyOutSignalCont_OFILM( void ) ;								// Slect Gyro Output Continuos Function
#ifdef STANDBY_MODE_OFILM
OISINI_OFILM__ void	AccWit_OFILM( unsigned char ) ;								// Acc Wait Function
OISINI_OFILM__ void	SelectGySleep_OFILM( unsigned char ) ;						// Select Gyro Mode Function
#endif
#ifdef	GAIN_CONT_OFILM
OISINI_OFILM__ void	AutoGainControlSw_OFILM( unsigned char ) ;							// Auto Gain Control Sw
#endif
OISINI_OFILM__ void	DrvSw_OFILM( unsigned char UcDrvSw ) ;						// Driver Mode setting function
OISINI_OFILM__ void	AfDrvSw_OFILM( unsigned char UcDrvSw ) ;						// AF Driver Mode setting function
OISINI_OFILM__ void	RamAccFixMod_OFILM( unsigned char ) ;							// Ram Access Fix Mode setting function
OISINI_OFILM__ void	IniPtMovMod_OFILM( unsigned char ) ;							// Pan/Tilt parameter setting by mode function
OISINI_OFILM__ void	ChkCvr_OFILM( void ) ;													// Check Function
	
OISCMD_OFILM__ void			SrvCon_OFILM( unsigned char, unsigned char ) ;					// Servo ON_OFILM/OFF_OFILM
OISCMD_OFILM__ unsigned short	TneRun_OFILM( void ) ;											// Hall System Auto Adjustment Function
OISCMD_OFILM__ unsigned char	RtnCen_OFILM( unsigned char ) ;									// Return to Center Function
OISCMD_OFILM__ void			OisEna_OFILM( void ) ;											// OIS Enable Function
OISCMD_OFILM__ void			OisEnaLin_OFILM( void ) ;											// OIS Enable Function for Line adjustment
OISCMD_OFILM__ void			TimPro_OFILM( void ) ;											// Timer Interrupt Process Function
OISCMD_OFILM__ void			S2cPro_OFILM( unsigned char ) ;									// S2 Command Process Function
	#define		DIFIL_S2_OFILM		0x3F7FFE00
OISCMD_OFILM__ void			SetSinWavePara_OFILM( unsigned char , unsigned char ) ;			// Sin wave Test Function
	#define		SINEWAVE_OFILM	0
	#define		XHALWAVE_OFILM	1
	#define		YHALWAVE_OFILM	2
	#define		XACTTEST_OFILM	10
	#define		YACTTEST_OFILM	11
	#define		CIRCWAVE_OFILM	255
OISCMD_OFILM__ unsigned char	TneGvc_OFILM( void ) ;											// Gyro VC Offset Adjust

OISCMD_OFILM__ void			SetZsp_OFILM( unsigned char ) ;									// Set Zoom Step parameter Function
OISCMD_OFILM__ void			OptCen_OFILM( unsigned char, unsigned short, unsigned short ) ;	// Set Optical Center adjusted value Function
OISCMD_OFILM__ void			StbOnnN_OFILM( unsigned char , unsigned char ) ;					// Stabilizer For Servo On Function
#ifdef	MODULE_CALIBRATION_OFILM
OISCMD_OFILM__ unsigned char	LopGan_OFILM( unsigned char ) ;									// Loop Gain Adjust
#endif
#ifdef STANDBY_MODE_OFILM
 OISCMD_OFILM__ void			SetStandby_OFILM( unsigned char ) ;								/* Standby control	*/
#endif
#ifdef	MODULE_CALIBRATION_OFILM
OISCMD_OFILM__ unsigned short	OscAdj_OFILM( void ) ;											/* OSC clock adjustment */
#endif

#ifdef	HALLADJ_HW_OFILM
 #ifdef	MODULE_CALIBRATION_OFILM
 OISCMD_OFILM__ unsigned char	LoopGainAdj_OFILM(   unsigned char );
 #endif
 OISCMD_OFILM__ unsigned char	BiasOffsetAdj_OFILM( unsigned char , unsigned char );
#endif
OISCMD_OFILM__ void			GyrGan_OFILM( unsigned char , unsigned long , unsigned long ) ;	/* Set Gyro Gain Function */
OISCMD_OFILM__ void			SetPanTiltMode_OFILM( unsigned char ) ;							/* Pan_Tilt control Function */
#ifndef	HALLADJ_HW_OFILM
 OISCMD_OFILM__ unsigned long	TnePtp_OFILM( unsigned char, unsigned char ) ;					// Get Hall Peak to Peak Values
	#define		HALL_H_VAL_OFILM	0x3F666666			/* 0.9 */
 OISCMD_OFILM__ unsigned char	TneCen_OFILM( unsigned char, UnDwdVal_OFILM ) ;							// Tuning Hall Center
 #define		PTP_BEFORE_OFILM		0
 #define		PTP_AFTER_OFILM		1
 #define		PTP_ACCEPT_OFILM		2
#endif
#ifdef GAIN_CONT_OFILM
OISCMD_OFILM__ unsigned char	TriSts_OFILM( void ) ;													// Read Status of Tripod mode Function
#endif
OISCMD_OFILM__ unsigned char	DrvPwmSw_OFILM( unsigned char ) ;											// Select Driver mode Function
	#define		Mlnp_OFILM		0					// Linear PWM
	#define		Mpwm_OFILM		1					// PWM
 #ifdef	NEUTRAL_CENTER_OFILM											// Gyro VC Offset Adjust
 OISCMD_OFILM__ unsigned char	TneHvc_OFILM( void ) ;											// Hall VC Offset Adjust
 #endif	//NEUTRAL_CENTER_OFILM
OISCMD_OFILM__ void			SetGcf_OFILM( unsigned char ) ;									// Set DI filter coefficient Function
OISCMD_OFILM__	unsigned long	UlH1Coefval_OFILM ;		// H1 coefficient value
#ifdef H1COEF_CHANGER_OFILM
 OISCMD_OFILM__	unsigned char	UcH1LvlMod_OFILM ;		// H1 level coef mode
 OISCMD_OFILM__	void			SetH1cMod_OFILM( unsigned char ) ;								// Set H1C coefficient Level chang Function
 #define		S2MODE_OFILM		0x40
 #define		ACTMODE_OFILM		0x80
 #define		MOVMODE_OFILM		0xFF
#endif
OISCMD_OFILM__	unsigned short	RdFwVr_OFILM( void ) ;										// Read Fw Version Function

#ifdef	ACCEPTANCE_OFILM
OISCMD_OFILM__	unsigned char	RunHea_OFILM( void ) ;										// Hall Examination of Acceptance
 #define		ACT_CHK_LVL_OFILM		0x3ECCCCCD		// 0.4
 #define		ACT_THR_OFILM			0x0400			// 28dB 20log(4/(0.4*256))
OISCMD_OFILM__	unsigned char	RunGea_OFILM( void ) ;										// Gyro Examination of Acceptance
 #define		GEA_DIF_HIG_OFILM		0x0010
 #define		GEA_DIF_LOW_OFILM		0x0001
#endif	//ACCEPTANCE_OFILM
// Dead Lock Check
OISCMD_OFILM__	unsigned char CmdRdChk_OFILM( void );
#define READ_COUNT_NUM_OFILM	3
void RegWriteA_Ofilm(unsigned short RegAddr, unsigned char RegData);
void RegReadA_Ofilm(unsigned short RegAddr, unsigned char *RegData);
void RamWriteA_Ofilm( unsigned short RamAddr, unsigned short RamData );
void RamReadA_Ofilm( unsigned short RamAddr, void * ReadData );
void RamWrite32A_Ofilm(unsigned short RamAddr, unsigned long RamData );
void RamRead32A_Ofilm(unsigned short RamAddr, void * ReadData );
void WitTim_Ofilm(unsigned short  UsWitTim_Ofilm );
void SeqWriteA_Ofilm( unsigned char * pData, unsigned short lens );

