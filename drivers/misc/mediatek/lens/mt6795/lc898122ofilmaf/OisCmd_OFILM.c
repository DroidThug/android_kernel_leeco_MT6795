//********************************************************************************
//
//		<< LC898122 Evaluation Soft >>
//	    Program Name	: OisCmd.c
//		Design			: Y.Yamada
//		History			: First edition						2009.07.31 Y.Tashita
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISCMD_OFILM

//#include	"Main.h"
//#include	"Cmd.h"
#include	"Ois_OFILM.h"
#include	"OisDef_OFILM.h"

//**************************
//	Local Function Prottype	
//**************************
void			MesFil_OFILM( unsigned char ) ;					// Measure Filter Setting
#ifdef	MODULE_CALIBRATION_OFILM
 #ifndef	HALLADJ_HW_OFILM
 void			LopIni_OFILM( unsigned char ) ;					// Loop Gain Initialize
 #endif
 void			LopPar_OFILM( unsigned char ) ;					// Loop Gain Parameter initialize
 #ifndef	HALLADJ_HW_OFILM
  void			LopSin_OFILM( unsigned char, unsigned char ) ;	// Loop Gain Sin Wave Output
  unsigned char	LopAdj_OFILM( unsigned char ) ;					// Loop Gain Adjust
  void			LopMes_OFILM( void ) ;							// Loop Gain Measure
 #endif
#endif
#ifndef	HALLADJ_HW_OFILM
unsigned long	GinMes_OFILM( unsigned char ) ;					// Measure Result Getting
#endif
void			GyrCon_OFILM( unsigned char ) ;					// Gyro Filter Control
short			GenMes_OFILM( unsigned short, unsigned char ) ;	// General Measure
#ifndef	HALLADJ_HW_OFILM
// unsigned long	TnePtp_OFILM( unsigned char, unsigned char ) ;	// Get Hall Peak to Peak Values
// unsigned char	TneCen_OFILM( unsigned char, UnDwdVal_OFILM ) ;			// Tuning Hall Center
 unsigned long	TneOff_OFILM( UnDwdVal_OFILM, unsigned char ) ;			// Hall Offset Tuning
 unsigned long	TneBia_OFILM( UnDwdVal_OFILM, unsigned char ) ;			// Hall Bias Tuning
#endif

void 			StbOnn_OFILM( void ) ;							// Servo ON_OFILM Slope mode

void			SetSineWave_OFILM(   unsigned char , unsigned char );
void			StartSineWave_OFILM( void );
void			StopSineWave_OFILM(  void );

void			SetMeasFil_OFILM(  unsigned char );
void			ClrMeasFil_OFILM( void );
unsigned char	TstActMov_OFILM( unsigned char );



//**************************
//	define					
//**************************
#define		MES_XG1_OFILM			0								// LXG1 Measure Mode
#define		MES_XG2_OFILM			1								// LXG2 Measure Mode

#define		HALL_ADJ_OFILM		0
#define		LOOPGAIN_OFILM		1
#define		THROUGH_OFILM			2
#define		NOISE_OFILM			3

// Measure Mode

 #define		TNE_OFILM 			80								// Waiting Time For Movement
#ifdef	HALLADJ_HW_OFILM

 #define     __MEASURE_LOOPGAIN_OFILM      0x00
 #define     __MEASURE_BIASOFFSET_OFILM    0x01

#else

 /******* Hall calibration Type 1 *******/
 #define		MARJIN_OFILM			0x0300							// Marjin
 #define		BIAS_ADJ_BORDER_OFILM	0x1998							// HALL_MAX_GAP < BIAS_ADJ_BORDER < HALL_MIN_GAP(80%)

 #define		HALL_MAX_GAP_OFILM	BIAS_ADJ_BORDER_OFILM - MARJIN_OFILM
 #define		HALL_MIN_GAP_OFILM	BIAS_ADJ_BORDER_OFILM + MARJIN_OFILM
 /***************************************/
 
 #define		BIAS_LIMIT_OFILM		0xFFFF							// HALL BIAS LIMIT
 #define		OFFSET_DIV_OFILM		2								// Divide Difference For Offset Step
 #define		TIME_OUT_OFILM		40								// Time Out Count

 /******* Hall calibration Type 2 *******/
 #define		MARGIN_OFILM			0x0300							// Margin

 #define		BIAS_ADJ_OVER_OFILM	0xD998							// 85%
 #define		BIAS_ADJ_RANGE_OFILM	0xCCCC							// 80%
 #define		BIAS_ADJ_SKIP_OFILM	0xBFFF							// 75%
 #define		HALL_MAX_RANGE_OFILM	BIAS_ADJ_RANGE_OFILM + MARGIN_OFILM
 #define		HALL_MIN_RANGE_OFILM	BIAS_ADJ_RANGE_OFILM - MARGIN_OFILM

 #define		DECRE_CAL_OFILM		0x0100							// decrease value
 /***************************************/
#endif

#ifdef H1COEF_CHANGER_OFILM
 #ifdef	CORRECT_1DEG_OFILM
//  #define		MAXLMT_OFILM		0x40400000				// 3.0
//  #define		MINLMT_OFILM		0x3FE66666				// 1.8
//  #define		CHGCOEF_OFILM		0xBA195555				// 
  #define		MAXLMT_OFILM		0x40600000				// 3.5  // modified by abe 140814
  #define		MINLMT_OFILM		0x400CCCCD				// 2.2	// modified by abe 140814
  #define		CHGCOEF_OFILM		0xBA0D89D9				// 

  #define		MINLMT_MOV_OFILM	0x00000000				// 0.0
  #define		CHGCOEF_MOV_OFILM	0xB8A49249

 #else
  #define		MAXLMT_OFILM		0x40000000				// 2.0
  #define		MINLMT_OFILM		0x3F8CCCCD				// 1.1
  #define		CHGCOEF_OFILM		0xBA4C71C7				// 

  #define		MINLMT_MOV_OFILM	0x00000000				// 0.0
  #define		CHGCOEF_MOV_OFILM	0xB9700000
 #endif
#endif

//**************************
//	Global Variable			
//**************************
#ifdef	HALLADJ_HW_OFILM
 unsigned char UcAdjBsy_OFILM;

#else
 unsigned short	UsStpSiz_OFILM	= 0 ;							// Bias Step Size
 unsigned short	UsErrBia_OFILM, UsErrOfs_OFILM ;
#endif



//**************************
//	Const					
//**************************
// gxzoom_OFILM Setting Value
#define		ZOOMTBL_OFILM	16
const unsigned long	ClGyxZom_OFILM[ ZOOMTBL_OFILM ]	= {
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000
	} ;

// gyzoom_OFILM Setting Value
const unsigned long	ClGyyZom_OFILM[ ZOOMTBL_OFILM ]	= {
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000,
		0x3F800000
	} ;

// DI Coefficient Setting Value
#define		COEFTBL_OFILM	7
const unsigned long	ClDiCof_OFILM[ COEFTBL_OFILM ]	= {
		DIFIL_S2_OFILM,		/* 0 */
		DIFIL_S2_OFILM,		/* 1 */
		DIFIL_S2_OFILM,		/* 2 */
		DIFIL_S2_OFILM,		/* 3 */
		DIFIL_S2_OFILM,		/* 4 */
		DIFIL_S2_OFILM,		/* 5 */
		DIFIL_S2_OFILM		/* 6 */
	} ;

#define TRACE(x...) 

//********************************************************************************
// Function Name 	: TneRun_OFILM
// Retun Value		: Hall Tuning SUCCESS_OFILM or FAILURE_OFILM
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned short	TneRun_OFILM( void )
{
	unsigned char	UcHlySts_OFILM, UcHlxSts_OFILM, UcAtxSts_OFILM, UcAtySts_OFILM ;
	unsigned short	UsFinSts_OFILM , UsOscSts_OFILM ; 								// Final Adjustment state
	unsigned char	UcDrvMod_OFILM ;
#ifndef	HALLADJ_HW_OFILM
	UnDwdVal_OFILM		StTneVal_OFILM ;
#endif

#ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
	UsOscSts_OFILM	= EXE_END_OFILM ;
#else
 #ifdef	MODULE_CALIBRATION_OFILM
	/* OSC adjustment */
	UsOscSts_OFILM	= OscAdj_OFILM() ;
 #else
	UsOscSts_OFILM	= EXE_END_OFILM ;
 #endif
#endif
	
	UcDrvMod_OFILM = UcPwmMod_OFILM ;
	if( UcDrvMod_OFILM == PWMMOD_CVL_OFILM )
	{
		DrvPwmSw_OFILM( Mpwm_OFILM ) ;		/* PWM mode */
	}
	
#ifdef	HALLADJ_HW_OFILM
	UcHlySts_OFILM = BiasOffsetAdj_OFILM( Y_DIR_OFILM , 0 ) ;
	WitTim_Ofilm( TNE_OFILM ) ;
	UcHlxSts_OFILM = BiasOffsetAdj_OFILM( X_DIR_OFILM , 0 ) ;
	WitTim_Ofilm( TNE_OFILM ) ;
	UcHlySts_OFILM = BiasOffsetAdj_OFILM( Y_DIR_OFILM , 1 ) ;
	WitTim_Ofilm( TNE_OFILM ) ;
	UcHlxSts_OFILM = BiasOffsetAdj_OFILM( X_DIR_OFILM , 1 ) ;

	SrvCon_OFILM( Y_DIR_OFILM, OFF_OFILM ) ;
	SrvCon_OFILM( X_DIR_OFILM, OFF_OFILM ) ;
	
	if( UcDrvMod_OFILM == PWMMOD_CVL_OFILM )
	{
		DrvPwmSw_OFILM( Mlnp_OFILM ) ;		/* PWM mode */
	}
	
  #ifdef	NEUTRAL_CENTER_OFILM
	TneHvc_OFILM();
  #endif	//NEUTRAL_CENTER_OFILM	
#else
//	StbOnnN_OFILM( OFF_OFILM , ON_OFILM ) ;				/* Y OFF_OFILM, X ON_OFILM */
	WitTim_Ofilm( TNE_OFILM ) ;

	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( Y_DIR_OFILM , PTP_BEFORE_OFILM ) ;
//	UcHlySts_OFILM	= TneCen_OFILM( Y_DIR_OFILM, StTneVal_OFILM ) ;
	UcHlySts_OFILM	= TneCen_OFILM( Y2_DIR_OFILM, StTneVal_OFILM ) ;
	
	StbOnnN_OFILM( ON_OFILM , OFF_OFILM ) ;				/* Y ON_OFILM, X OFF_OFILM */
	WitTim_Ofilm( TNE_OFILM ) ;

	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( X_DIR_OFILM , PTP_BEFORE_OFILM ) ;
//	UcHlxSts_OFILM	= TneCen_OFILM( X_DIR_OFILM, StTneVal_OFILM ) ;
	UcHlxSts_OFILM	= TneCen_OFILM( X2_DIR_OFILM, StTneVal_OFILM ) ;

	StbOnnN_OFILM( OFF_OFILM , ON_OFILM ) ;				/* Y OFF_OFILM, X ON_OFILM */
	WitTim_Ofilm( TNE_OFILM ) ;

	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( Y_DIR_OFILM , PTP_AFTER_OFILM ) ;
//	UcHlySts_OFILM	= TneCen_OFILM( Y_DIR_OFILM, StTneVal_OFILM ) ;
	UcHlySts_OFILM	= TneCen_OFILM( Y2_DIR_OFILM, StTneVal_OFILM ) ;

	StbOnnN_OFILM( ON_OFILM , OFF_OFILM ) ;				/* Y ON_OFILM, X OFF_OFILM */
	WitTim_Ofilm( TNE_OFILM ) ;

	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( X_DIR_OFILM , PTP_AFTER_OFILM ) ;
//	UcHlxSts_OFILM	= TneCen_OFILM( X_DIR_OFILM, StTneVal_OFILM ) ;
	UcHlxSts_OFILM	= TneCen_OFILM( X2_DIR_OFILM, StTneVal_OFILM ) ;

	SrvCon_OFILM( Y_DIR_OFILM, OFF_OFILM ) ;
	SrvCon_OFILM( X_DIR_OFILM, OFF_OFILM ) ;
	
	if( UcDrvMod_OFILM == PWMMOD_CVL_OFILM )
	{
		DrvPwmSw_OFILM( Mlnp_OFILM ) ;		/* PWM mode */
	}
	
  #ifdef	NEUTRAL_CENTER_OFILM
	TneHvc_OFILM();
  #endif	//NEUTRAL_CENTER_OFILM
#endif
	

	WitTim_Ofilm( TNE_OFILM ) ;

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	StAdjPar_OFILM.StHalAdj_OFILM.UsAdxOff_OFILM = (unsigned short)((unsigned long)0x00010000 - (unsigned long)StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM ) ;
	StAdjPar_OFILM.StHalAdj_OFILM.UsAdyOff_OFILM = (unsigned short)((unsigned long)0x00010000 - (unsigned long)StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM ) ;
TRACE("    Xadof = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsAdxOff_OFILM ) ;
TRACE("    Yadof = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsAdyOff_OFILM ) ;
	
	RamWriteA_Ofilm( OFF0Z_OFILM,  StAdjPar_OFILM.StHalAdj_OFILM.UsAdxOff_OFILM ) ;	// 0x1450
	RamWriteA_Ofilm( OFF1Z_OFILM,  StAdjPar_OFILM.StHalAdj_OFILM.UsAdyOff_OFILM ) ;	// 0x14D0

	RamReadA_Ofilm( DAXHLO_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlxOff_OFILM ) ;		// 0x1479
	RamReadA_Ofilm( DAXHLB_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlxGan_OFILM ) ;		// 0x147A
	RamReadA_Ofilm( DAYHLO_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyOff_OFILM ) ;		// 0x14F9
	RamReadA_Ofilm( DAYHLB_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyGan_OFILM ) ;		// 0x14FA
	RamReadA_Ofilm( OFF0Z_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsAdxOff_OFILM ) ;		// 0x1450
	RamReadA_Ofilm( OFF1Z_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsAdyOff_OFILM ) ;		// 0x14D0
	
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
	StbOnn_OFILM() ;											// Slope Mode

	
	WitTim_Ofilm( TNE_OFILM ) ;

#ifdef	MODULE_CALIBRATION_OFILM
	// X Loop Gain Adjust
	UcAtxSts_OFILM	= LopGan_OFILM( X_DIR_OFILM ) ;


	// Y Loop Gain Adjust
	UcAtySts_OFILM	= LopGan_OFILM( Y_DIR_OFILM ) ;
#else		//  default value
	RamAccFixMod_OFILM( ON_OFILM ) ;								// Fix mode
	RamReadA_Ofilm( sxg_OFILM, &StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM ) ;		// 0x10D3
	RamReadA_Ofilm( syg_OFILM, &StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM ) ;		// 0x11D3
	RamAccFixMod_OFILM( OFF_OFILM ) ;								// Float mode
	UcAtxSts_OFILM	= EXE_END_OFILM ;
	UcAtySts_OFILM	= EXE_END_OFILM ;
#endif
	

	TneGvc_OFILM() ;


	UsFinSts_OFILM	= (unsigned short)( UcHlxSts_OFILM - EXE_END_OFILM ) + (unsigned short)( UcHlySts_OFILM - EXE_END_OFILM ) + (unsigned short)( UcAtxSts_OFILM - EXE_END_OFILM ) + (unsigned short)( UcAtySts_OFILM - EXE_END_OFILM ) + ( UsOscSts_OFILM - (unsigned short)EXE_END_OFILM ) + (unsigned short)EXE_END_OFILM ;


	TRACE( "ADJ STS = %04X\n\n", UsFinSts_OFILM ) ;
	
/* \95\\8E\A6 */
TRACE(" OSC CLOCK Data **********> \n" ) ;
TRACE("    CLK VAL = %04xh \n\n", StAdjPar_OFILM.UcOscVal_OFILM ) ;
	
TRACE(" X Axis Data **********> \n" ) ;
TRACE("Hall CENT     = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCen_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM ) ;
TRACE("Hall X MAX    = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMax_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMxa_OFILM ) ;
TRACE("Hall X MIN    = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMin_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMna_OFILM ) ;
TRACE("Hall GAIN     = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxGan_OFILM ) ;
TRACE("Hall OFFSET   = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxOff_OFILM ) ;
TRACE("Hall AD OFST  = %04xh \n\n", StAdjPar_OFILM.StHalAdj_OFILM.UsAdxOff_OFILM ) ;
	
TRACE(" Y Axis Data **********> \n" ) ;
TRACE("Hall CENT     = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCen_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM ) ;
TRACE("Hall Y MAX    = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMax_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMxa_OFILM ) ;
TRACE("Hall Y MIN    = %04xh ", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMin_OFILM ) ;
TRACE("    AFTER = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMna_OFILM ) ;
TRACE("Hall GAIN     = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyGan_OFILM ) ;
TRACE("Hall OFFSET   = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyOff_OFILM ) ;
TRACE("Hall AD OFST  = %04xh \n\n", StAdjPar_OFILM.StHalAdj_OFILM.UsAdyOff_OFILM ) ;
	
TRACE(" X Axis Loop **********> \n" ) ;
TRACE("LoopG X 1st = %04xh \n", StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM ) ;
TRACE(" Y Axis Loop **********> \n" ) ;
TRACE("LoopG Y 1st = %04xh \n\n", StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM ) ;
	
TRACE(" X Axis GYRO **********> \n" ) ;
TRACE("Gyro X 1st  = %04xh \n", StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM ) ;
TRACE(" Y Axis GYRO **********> \n" ) ;
TRACE("Gyro Y 1st  = %04xh \n\n", StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM ) ;
	
	return( UsFinSts_OFILM ) ;
}


#ifndef	HALLADJ_HW_OFILM

//********************************************************************************
// Function Name 	: TnePtp_OFILM
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
 
unsigned long	TnePtp_OFILM ( unsigned char	UcDirSel_OFILM, unsigned char	UcBfrAft_OFILM )
{
	UnDwdVal_OFILM		StTneVal_OFILM ;

	MesFil_OFILM( THROUGH_OFILM ) ;					// B


	if ( !UcDirSel_OFILM ) {
		RamWrite32A_Ofilm( sxsin_OFILM , HALL_H_VAL_OFILM );		// 0x10D5
		SetSinWavePara_OFILM( 0x0A , XHALWAVE_OFILM ); 
	}else{
	    RamWrite32A_Ofilm( sysin_OFILM , HALL_H_VAL_OFILM ); 		// 0x11D5
		SetSinWavePara_OFILM( 0x0A , YHALWAVE_OFILM ); 
	}

	if ( !UcDirSel_OFILM ) {					// AXIS X
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )AD0Z_OFILM ) ;							/* 0x0194	*/
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( AD0Z_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
	} else {							// AXIS Y
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )AD1Z_OFILM ) ;							/* 0x0194	*/
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( AD1Z_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
	}

	RegWriteA_Ofilm( WC_MESLOOP1_OFILM	, 0x00 );			// 0x0193	CmMesLoop[15:8]
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM	, 0x01);			// 0x0192	CmMesLoop[7:0]
	
	RamWrite32A_Ofilm( msmean_OFILM	, 0x3F800000 );			// 0x1230	1/CmMesLoop[15:0]
	
	RamWrite32A_Ofilm( MSMAX1_OFILM, 	0x00000000 ) ;		// 0x1050
	RamWrite32A_Ofilm( MSMAX1AV_OFILM, 	0x00000000 ) ;		// 0x1051
	RamWrite32A_Ofilm( MSMIN1_OFILM, 	0x00000000 ) ;		// 0x1060
	RamWrite32A_Ofilm( MSMIN1AV_OFILM, 	0x00000000 ) ;		// 0x1061
	
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x00 ) ;				// 0x0198	none ABS
	BsyWit_OFILM( WC_MESMODE_OFILM, 0x02 ) ;				// 0x0190		Sine wave Measure

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	RamReadA_Ofilm( MSMAX1AV_OFILM, &StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ) ;		// 0x1051
	RamReadA_Ofilm( MSMIN1AV_OFILM, &StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) ;		// 0x1061

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode

	if ( !UcDirSel_OFILM ) {					// AXIS X
		SetSinWavePara_OFILM( 0x00 , XHALWAVE_OFILM ); 	/* STOP */
	}else{
		SetSinWavePara_OFILM( 0x00 , YHALWAVE_OFILM ); 	/* STOP */
	}

	if( UcBfrAft_OFILM == PTP_ACCEPT_OFILM ) {
		
TRACE("Hig = %04x, Low = %04x\n", StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM, StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) ;
		
		StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM	= 0x7fff - StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;		// Maximum Gap = Maximum - Hall Peak Top
		StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM - 0x7fff ; 	// Minimum Gap = Hall Peak Bottom - Minimum
		
TRACE("GapH = %04x, GapL = %04x\n", StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM, StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) ;
		
	} else {
		if( UcBfrAft_OFILM == 0 ) {
			if( UcDirSel_OFILM == X_DIR_OFILM ) {
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCen_OFILM	= ( ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMax_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMin_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ;
			} else {
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCen_OFILM	= ( ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMax_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMin_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ;
			}
		} else {
			if( UcDirSel_OFILM == X_DIR_OFILM ){
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM	= ( ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMxa_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMna_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ;
			} else {
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM	= ( ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMxa_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;
				StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMna_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ;
			}
		}
	}
	
TRACE("ADJ(%d) MAX = %04x, MIN = %04x, CNT = %04x, ", UcDirSel_OFILM, StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM, StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM, ( ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + ( signed short )StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ) ;
	StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM	= 0x7fff - StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ;		// Maximum Gap = Maximum - Hall Peak Top
	StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM	= StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM - 0x7fff ; 	// Minimum Gap = Hall Peak Bottom - Minimum

TRACE("GapH = %04x, GapL = %04x\n", StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM, StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) ;
	
	return( StTneVal_OFILM.UlDwdVal_OFILM ) ;
}

//********************************************************************************
// Function Name 	: TneCen_OFILM
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned short	UsValBef_OFILM,UsValNow_OFILM ;
unsigned char	TneCen_OFILM( unsigned char	UcTneAxs_OFILM, UnDwdVal_OFILM	StTneVal_OFILM )
{
	unsigned char 	UcTneRst_OFILM, UcTmeOut_OFILM, UcTofRst_OFILM ;
	unsigned short	UsOffDif_OFILM ;
	unsigned short	UsBiasVal_OFILM ;

	UsErrBia_OFILM	= 0 ;
	UsErrOfs_OFILM	= 0 ;
	UcTmeOut_OFILM	= 1 ;
	UsStpSiz_OFILM	= 1 ;
	UcTneRst_OFILM	= FAILURE_OFILM ;
	UcTofRst_OFILM	= FAILURE_OFILM ;

	while ( UcTneRst_OFILM && UcTmeOut_OFILM )
	{
		if( UcTofRst_OFILM == FAILURE_OFILM ) {
			StTneVal_OFILM.UlDwdVal_OFILM	= TneOff_OFILM( StTneVal_OFILM, UcTneAxs_OFILM ) ;
		} else {
			StTneVal_OFILM.UlDwdVal_OFILM	= TneBia_OFILM( StTneVal_OFILM, UcTneAxs_OFILM ) ;
			UcTofRst_OFILM	= FAILURE_OFILM ;
		}

		if( !( UcTneAxs_OFILM & 0xF0 ) )
		{
			if ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM > StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) {									// Check Offset Tuning Result
				UsOffDif_OFILM	= ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM - StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 ;
			} else {
				UsOffDif_OFILM	= ( StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM - StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ) / 2 ;
			}

			if( UsOffDif_OFILM < MARJIN_OFILM ) {
				UcTofRst_OFILM	= SUCCESS_OFILM ;
			} else {
				UcTofRst_OFILM	= FAILURE_OFILM ;
			}

			if ( ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM < HALL_MIN_GAP_OFILM&& StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM < HALL_MIN_GAP_OFILM)		// Check Tuning Result 
			&& ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM > HALL_MAX_GAP_OFILM&& StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM > HALL_MAX_GAP_OFILM) ) {
				UcTneRst_OFILM	= SUCCESS_OFILM ;
				break ;
			} else if ( UsStpSiz_OFILM == 0 ) {
				UcTneRst_OFILM	= SUCCESS_OFILM ;
				break ;
			} else {
				UcTneRst_OFILM	= FAILURE_OFILM ;
				UcTmeOut_OFILM++ ;
			}
		}else{
TRACE("  No = %04d", UcTmeOut_OFILM ) ;
			if( (StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM > MARGIN_OFILM) && (StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM > MARGIN_OFILM) )	/* position check */
			{
				UcTofRst_OFILM	= SUCCESS_OFILM ;
TRACE("  TofR = SUCC" ) ;
				UsValBef_OFILM = UsValNow_OFILM = 0x0000 ;
			}else if( (StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM <= MARGIN_OFILM) && (StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM <= MARGIN_OFILM) ){
				UcTofRst_OFILM	= SUCCESS_OFILM ;
				UcTneRst_OFILM	= FAILURE_OFILM ;
TRACE("  TofR = SUCC   But TneR = Fail" ) ;
			}else if( ((unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) > BIAS_ADJ_OVER_OFILM ) {
				UcTofRst_OFILM	= SUCCESS_OFILM ;
				UcTneRst_OFILM	= FAILURE_OFILM ;
TRACE("  TofR = SUCC   But TneR = Fail" ) ;
			}else{
				UcTofRst_OFILM	= FAILURE_OFILM ;

TRACE("  TofR = FAIL" ) ;
				
				UsValBef_OFILM = UsValNow_OFILM ;

				RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
				
				if( !( UcTneAxs_OFILM & 0x0F ) ) {
					RamReadA_Ofilm( DAXHLO_OFILM, &UsValNow_OFILM ) ;				// 0x1479	Hall X Offset Read
				}else{
					RamReadA_Ofilm( DAYHLO_OFILM, &UsValNow_OFILM ) ;				// 0x14F9	Hall Y Offset Read
				}
				if( ((( UsValBef_OFILM & 0xFF00 ) == 0x8000 ) && ( UsValNow_OFILM & 0xFF00 ) == 0x8000 )
				 || ((( UsValBef_OFILM & 0xFF00 ) == 0x7F00 ) && ( UsValNow_OFILM & 0xFF00 ) == 0x7F00 ) )
				{
					if( !( UcTneAxs_OFILM & 0x0F ) ) {
						RamReadA_Ofilm( DAXHLB_OFILM, &UsBiasVal_OFILM ) ;		// 0x147A	Hall X Bias Read
TRACE("	CENT:Bf-Dat DAXHLB_OFILM = %04x",  UsBiasVal_OFILM ) ;
					}else{
						RamReadA_Ofilm( DAYHLB_OFILM, &UsBiasVal_OFILM ) ;		// 0x14FA	Hall Y Bias Read
TRACE("	CENT:Bf-Dat DAYHLB_OFILM = %04x",  UsBiasVal_OFILM ) ;
					}
					if( UsBiasVal_OFILM > 0x8000 )
					{
						UsBiasVal_OFILM -= 0x8000 ;
					}
					else
					{
						UsBiasVal_OFILM += 0x8000 ;
					}
					if( UsBiasVal_OFILM > DECRE_CAL_OFILM )
					{
						UsBiasVal_OFILM -= DECRE_CAL_OFILM ;
					}
					UsBiasVal_OFILM += 0x8000 ;
					
					if( !( UcTneAxs_OFILM & 0x0F ) ) {
						RamWriteA_Ofilm( DAXHLB_OFILM, UsBiasVal_OFILM ) ;		// 0x147A	Hall X Bias
TRACE("	=>	Af-Dat DAXHLB_OFILM = %04x\n",  UsBiasVal_OFILM ) ;
					}else{
						RamWriteA_Ofilm( DAYHLB_OFILM, UsBiasVal_OFILM ) ;		// 0x14FA	Hall Y Bias
TRACE("	=>	Af-Dat DAYHLB_OFILM = %04x\n",  UsBiasVal_OFILM ) ;
					}
				}

				RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
				
			}
			
			if((( (unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) < HALL_MAX_RANGE_OFILM)
			&& (( (unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) > HALL_MIN_RANGE_OFILM) ) {
				if(UcTofRst_OFILM	== SUCCESS_OFILM)
				{
					UcTneRst_OFILM	= SUCCESS_OFILM ;
TRACE("  TneaN = SUCC\n" ) ;
					break ;
				}
			}
			UcTneRst_OFILM	= FAILURE_OFILM ;
			UcTmeOut_OFILM++ ;
TRACE("  Tne = FAIL\n" ) ;
		}

		if( UcTneAxs_OFILM & 0xF0 )
		{
			if ( ( UcTmeOut_OFILM / 2 ) == TIME_OUT_OFILM ) {
				UcTmeOut_OFILM	= 0 ;
			}		 																							// Set Time Out Count
		}else{
			if ( UcTmeOut_OFILM == TIME_OUT_OFILM ) {
				UcTmeOut_OFILM	= 0 ;
			}		 																							// Set Time Out Count
		}
	}

	if( UcTneRst_OFILM == FAILURE_OFILM ) {
		if( !( UcTneAxs_OFILM & 0x0F ) ) {
			UcTneRst_OFILM					= EXE_HXADJ_OFILM ;
			StAdjPar_OFILM.StHalAdj_OFILM.UsHlxGan_OFILM	= 0xFFFF ;
			StAdjPar_OFILM.StHalAdj_OFILM.UsHlxOff_OFILM	= 0xFFFF ;
		} else {
			UcTneRst_OFILM					= EXE_HYADJ_OFILM ;
			StAdjPar_OFILM.StHalAdj_OFILM.UsHlyGan_OFILM	= 0xFFFF ;
			StAdjPar_OFILM.StHalAdj_OFILM.UsHlyOff_OFILM	= 0xFFFF ;
		}
	} else {
		UcTneRst_OFILM	= EXE_END_OFILM ;
	}

	return( UcTneRst_OFILM ) ;
}



//********************************************************************************
// Function Name 	: TneBia_OFILM
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned long	TneBia_OFILM( UnDwdVal_OFILM	StTneVal_OFILM, unsigned char	UcTneAxs_OFILM )
{
	long					SlSetBia_OFILM ;
	unsigned short			UsSetBia_OFILM ;
	unsigned char			UcChkFst_OFILM ;
	static unsigned short	UsTneVax_OFILM ;							// Variable For 1/2 Searching
	unsigned short			UsDecCal_OFILM ;

	UcChkFst_OFILM	= 1 ;

	if ( UsStpSiz_OFILM == 1) {
		UsTneVax_OFILM	= 2 ;

		if( UcTneAxs_OFILM & 0xF0 ){
			if ( ((unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) > BIAS_ADJ_OVER_OFILM ) {
				UcChkFst_OFILM	= 0 ;

				RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
				
				if ( !( UcTneAxs_OFILM & 0x0F ) ) {							// Initializing Hall Offset & Bias, Step Size
					RamReadA_Ofilm( DAXHLB_OFILM, &UsSetBia_OFILM ) ;		// 0x147A	Hall X Bias Read
TRACE("	BIAS:Bf-Dat DAXHLB_OFILM = %04x",  UsSetBia_OFILM ) ;
				} else {
					RamReadA_Ofilm( DAYHLB_OFILM, &UsSetBia_OFILM ) ;		// 0x14FA	Hall Y Bias Read
TRACE("	BIAS:Bf-Dat DAYHLB_OFILM = %04x",  UsSetBia_OFILM ) ;
				}
				if( UsSetBia_OFILM > 0x8000 )
				{
					UsSetBia_OFILM -= 0x8000 ;
				}
				else
				{
					UsSetBia_OFILM += 0x8000 ;
				}
				if( !UcChkFst_OFILM )	{
					UsDecCal_OFILM = ( DECRE_CAL_OFILM << 3 ) ;
				}else{
					UsDecCal_OFILM = DECRE_CAL_OFILM ;
				}
				if( UsSetBia_OFILM > UsDecCal_OFILM )
				{
					UsSetBia_OFILM -= UsDecCal_OFILM ;
				}
				UsSetBia_OFILM += 0x8000 ;
				if ( !( UcTneAxs_OFILM & 0x0F ) ) {							// Initializing Hall Offset & Bias, Step Size
					RamWriteA_Ofilm( DAXHLB_OFILM, UsSetBia_OFILM ) ;				// 0x147A	Hall X Bias
TRACE("	=>	Af-Dat DAXHLB_OFILM = %04x\n",  UsSetBia_OFILM ) ;
					RamWriteA_Ofilm( DAXHLO_OFILM, 0x0000 ) ;				// 0x1479	Hall X Offset 0x0000
				} else {
					RamWriteA_Ofilm( DAYHLB_OFILM, UsSetBia_OFILM ) ;				// 0x14FA	Hall Y Bias
TRACE("	=>	Af-Dat DAYHLB_OFILM = %04x\n",  UsSetBia_OFILM ) ;
					RamWriteA_Ofilm( DAYHLO_OFILM, 0x0000 ) ;				// 0x14F9	Hall Y Offset 0x0000
				}
				UsStpSiz_OFILM	= BIAS_LIMIT_OFILM / UsTneVax_OFILM ;

				RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
				
			}
		}else{
			if ( ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 < BIAS_ADJ_BORDER_OFILM) {
				UcChkFst_OFILM	= 0 ;
			}

			if ( !UcTneAxs_OFILM ) {										// Initializing Hall Offset & Bias, Step Size
				
				RamWrite32A_Ofilm( DAXHLB_OFILM, 0xBF800000 ) ; 	// 0x147A	Hall X Bias 0x8001
				RamWrite32A_Ofilm( DAXHLO_OFILM, 0x00000000 ) ;		// 0x1479	Hall X Offset 0x0000

				UsStpSiz_OFILM	= BIAS_LIMIT_OFILM / UsTneVax_OFILM ;
			} else {
				RamWrite32A_Ofilm( DAYHLB_OFILM, 0xBF800000 ) ; 	// 0x14FA	Hall Y Bias 0x8001
				RamWrite32A_Ofilm( DAYHLO_OFILM, 0x00000000 ) ;		// 0x14F9	 Y Offset 0x0000
				UsStpSiz_OFILM	= BIAS_LIMIT_OFILM / UsTneVax_OFILM ;
			}
		}
	}

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	if ( !( UcTneAxs_OFILM & 0x0F ) ) {
		RamReadA_Ofilm( DAXHLB_OFILM, &UsSetBia_OFILM ) ;					// 0x147A	Hall X Bias Read
		SlSetBia_OFILM	= ( long )UsSetBia_OFILM ;
	} else {
		RamReadA_Ofilm( DAYHLB_OFILM, &UsSetBia_OFILM ) ;					// 0x14FA	Hall Y Bias Read
		SlSetBia_OFILM	= ( long )UsSetBia_OFILM ;
	}

	if( SlSetBia_OFILM >= 0x00008000 ) {
		SlSetBia_OFILM	|= 0xFFFF0000 ;
	}

	if( UcChkFst_OFILM ) {
		if( UcTneAxs_OFILM & 0xF0 )
		{
			if ( ((unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) < BIAS_ADJ_RANGE_OFILM ) {	// Calculatiton For Hall BIAS 1/2 Searching
				if( ((unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) < BIAS_ADJ_SKIP_OFILM ) {
					SlSetBia_OFILM	+= 0x0400 ;
				}else{
					SlSetBia_OFILM	+= 0x0100 ;
				}
			} else {
				if( ((unsigned short)0xFFFF - ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM )) > BIAS_ADJ_OVER_OFILM ) {
					SlSetBia_OFILM	-= 0x0400 ;
				}else{
					SlSetBia_OFILM	-= 0x0100 ;
				}
			}
			UsStpSiz_OFILM	= 0x0200 ;
			
		}else{
			if ( ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM + StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / 2 > BIAS_ADJ_BORDER_OFILM) {	// Calculatiton For Hall BIAS 1/2 Searching
				SlSetBia_OFILM	+= UsStpSiz_OFILM ;
			} else {
				SlSetBia_OFILM	-= UsStpSiz_OFILM ;
			}

			UsTneVax_OFILM	= UsTneVax_OFILM * 2 ;
			UsStpSiz_OFILM	= BIAS_LIMIT_OFILM / UsTneVax_OFILM ;
		}
	}

	if( SlSetBia_OFILM > ( long )0x00007FFF ) {
		SlSetBia_OFILM	= 0x00007FFF ;
	} else if( SlSetBia_OFILM < ( long )0xFFFF8001 ) {
		SlSetBia_OFILM	= 0xFFFF8001 ;
	}

	if ( !( UcTneAxs_OFILM & 0x0F ) ) {
		RamWriteA_Ofilm( DAXHLB_OFILM, SlSetBia_OFILM ) ;		// 0x147A	Hall X Bias Ram Write
TRACE("		DAXHLB_OFILM = %04x\n ",  (unsigned short)SlSetBia_OFILM ) ;
	} else {
		RamWriteA_Ofilm( DAYHLB_OFILM, SlSetBia_OFILM ) ;		// 0x14FA	Hall Y Bias Ram Write
TRACE("		DAYHLB_OFILM = %04x\n ",  (unsigned short)SlSetBia_OFILM ) ;
	}
	
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode

	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( UcTneAxs_OFILM & 0x0F , PTP_AFTER_OFILM ) ;

	return( StTneVal_OFILM.UlDwdVal_OFILM ) ;
}



//********************************************************************************
// Function Name 	: TneOff_OFILM
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned long	TneOff_OFILM( UnDwdVal_OFILM	StTneVal_OFILM, unsigned char	UcTneAxs_OFILM )
{
	long			SlSetOff_OFILM ;
	unsigned short	UsSetOff_OFILM ;

	UcTneAxs_OFILM &= 0x0F ;
	
	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
		
	if ( !UcTneAxs_OFILM ) {																			// Initializing Hall Offset & Bias
		RamReadA_Ofilm( DAXHLO_OFILM, &UsSetOff_OFILM ) ;				// 0x1479	Hall X Offset Read
		SlSetOff_OFILM	= ( long )UsSetOff_OFILM ;
	} else {
		RamReadA_Ofilm( DAYHLO_OFILM, &UsSetOff_OFILM ) ;				// 0x14F9	Hall Y Offset Read
		SlSetOff_OFILM	= ( long )UsSetOff_OFILM ;
	}

	if( SlSetOff_OFILM > 0x00008000 ) {
		SlSetOff_OFILM	|= 0xFFFF0000 ;
	}

	if ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM > StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) {
		SlSetOff_OFILM	+= ( StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM - StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM ) / OFFSET_DIV_OFILM;	// Calculating Value For Increase Step
	} else {
		SlSetOff_OFILM	-= ( StTneVal_OFILM.StDwdVal_OFILM.UsLowVal_OFILM - StTneVal_OFILM.StDwdVal_OFILM.UsHigVal_OFILM ) / OFFSET_DIV_OFILM;	// Calculating Value For Decrease Step
	}

	if( SlSetOff_OFILM > ( long )0x00007FFF ) {
		SlSetOff_OFILM	= 0x00007FFF ;
	} else if( SlSetOff_OFILM < ( long )0xFFFF8001 ) {
		SlSetOff_OFILM	= 0xFFFF8001 ;
	}

	if ( !UcTneAxs_OFILM ) {
		RamWriteA_Ofilm( DAXHLO_OFILM, SlSetOff_OFILM ) ;		// 0x1479	Hall X Offset Ram Write
TRACE("		DAXHLO_OFILM = %04x\n ",  (unsigned short)SlSetOff_OFILM ) ;
	} else {
		RamWriteA_Ofilm( DAYHLO_OFILM, SlSetOff_OFILM ) ;		// 0x14F9	Hall Y Offset Ram Write
TRACE("		DAYHLO_OFILM = %04x\n ",  (unsigned short)SlSetOff_OFILM ) ;
	}

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
	StTneVal_OFILM.UlDwdVal_OFILM	= TnePtp_OFILM( UcTneAxs_OFILM, PTP_AFTER_OFILM ) ;

	return( StTneVal_OFILM.UlDwdVal_OFILM ) ;
}

#endif

//********************************************************************************
// Function Name 	: MesFil_OFILM
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 						2009.07.31  Y.Tashita
//********************************************************************************
void	MesFil_OFILM( unsigned char	UcMesMod_OFILM )
{
#ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
	if( !UcMesMod_OFILM ) {								// Hall Bias&Offset Adjust
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3D1E5A40 ) ;		// 0x10F0	LPF150Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3D1E5A40 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F6C34C0 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F800000 ) ;		// 0x10F5	Through
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x00000000 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x00000000 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3D1E5A40 ) ;		// 0x11F0	LPF150Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3D1E5A40 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F6C34C0 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F800000 ) ;		// 0x11F5	Through
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x00000000 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x00000000 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == LOOPGAIN_OFILM ) {				// Loop Gain Adjust
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3E587E00 ) ;		// 0x10F0	LPF1000Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3E587E00 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F13C100 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F7DF500 ) ;		// 0x10F5	HPF30Hz
		RamWrite32A_Ofilm( mes1bb_OFILM, 0xBF7DF500 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x3F7BEA40 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3E587E00 ) ;		// 0x11F0	LPF1000Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3E587E00 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F13C100 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F7DF500 ) ;		// 0x11F5	HPF30Hz
		RamWrite32A_Ofilm( mes2bb_OFILM, 0xBF7DF500 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x3F7BEA40 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == THROUGH_OFILM ) {				// for Through
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3F800000 ) ;		// 0x10F0	Through
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x00000000 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x00000000 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F800000 ) ;		// 0x10F5	Through
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x00000000 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x00000000 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3F800000 ) ;		// 0x11F0	Through
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x00000000 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x00000000 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F800000 ) ;		// 0x11F5	Through
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x00000000 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x00000000 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == NOISE_OFILM ) {				// SINE WAVE TEST for NOISE_OFILM
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3D1E5A40 ) ;		// 0x10F0	LPF150Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3D1E5A40 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F6C34C0 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3D1E5A40 ) ;		// 0x10F5	LPF150Hz
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x3D1E5A40 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x3F6C34C0 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3D1E5A40 ) ;		// 0x11F0	LPF150Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3D1E5A40 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F6C34C0 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3D1E5A40 ) ;		// 0x11F5	LPF150Hz
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x3D1E5A40 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x3F6C34C0 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
	}
#else
	if( !UcMesMod_OFILM ) {								// Hall Bias&Offset Adjust
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3CA175C0 ) ;		// 0x10F0	LPF150Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3CA175C0 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F75E8C0 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F800000 ) ;		// 0x10F5	Through
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x00000000 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x00000000 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3CA175C0 ) ;		// 0x11F0	LPF150Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3CA175C0 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F75E8C0 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F800000 ) ;		// 0x11F5	Through
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x00000000 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x00000000 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == LOOPGAIN_OFILM ) {				// Loop Gain Adjust
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3DF21080 ) ;		// 0x10F0	LPF1000Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3DF21080 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F437BC0 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F7EF980 ) ;		// 0x10F5	HPF30Hz
		RamWrite32A_Ofilm( mes1bb_OFILM, 0xBF7EF980 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x3F7DF300 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3DF21080 ) ;		// 0x11F0	LPF1000Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3DF21080 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F437BC0 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F7EF980 ) ;		// 0x11F5	HPF30Hz
		RamWrite32A_Ofilm( mes2bb_OFILM, 0xBF7EF980 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x3F7DF300 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == THROUGH_OFILM ) {				// for Through
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3F800000 ) ;		// 0x10F0	Through
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x00000000 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x00000000 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3F800000 ) ;		// 0x10F5	Through
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x00000000 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x00000000 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3F800000 ) ;		// 0x11F0	Through
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x00000000 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x00000000 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3F800000 ) ;		// 0x11F5	Through
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x00000000 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x00000000 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
		
	} else if( UcMesMod_OFILM == NOISE_OFILM ) {				// SINE WAVE TEST for NOISE_OFILM
		// Measure Filter1 Setting
		RamWrite32A_Ofilm( mes1aa_OFILM, 0x3CA175C0 ) ;		// 0x10F0	LPF150Hz
		RamWrite32A_Ofilm( mes1ab_OFILM, 0x3CA175C0 ) ;		// 0x10F1
		RamWrite32A_Ofilm( mes1ac_OFILM, 0x3F75E8C0 ) ;		// 0x10F2
		RamWrite32A_Ofilm( mes1ad_OFILM, 0x00000000 ) ;		// 0x10F3
		RamWrite32A_Ofilm( mes1ae_OFILM, 0x00000000 ) ;		// 0x10F4
		RamWrite32A_Ofilm( mes1ba_OFILM, 0x3CA175C0 ) ;		// 0x10F5	LPF150Hz
		RamWrite32A_Ofilm( mes1bb_OFILM, 0x3CA175C0 ) ;		// 0x10F6
		RamWrite32A_Ofilm( mes1bc_OFILM, 0x3F75E8C0 ) ;		// 0x10F7
		RamWrite32A_Ofilm( mes1bd_OFILM, 0x00000000 ) ;		// 0x10F8
		RamWrite32A_Ofilm( mes1be_OFILM, 0x00000000 ) ;		// 0x10F9
		
		// Measure Filter2 Setting
		RamWrite32A_Ofilm( mes2aa_OFILM, 0x3CA175C0 ) ;		// 0x11F0	LPF150Hz
		RamWrite32A_Ofilm( mes2ab_OFILM, 0x3CA175C0 ) ;		// 0x11F1
		RamWrite32A_Ofilm( mes2ac_OFILM, 0x3F75E8C0 ) ;		// 0x11F2
		RamWrite32A_Ofilm( mes2ad_OFILM, 0x00000000 ) ;		// 0x11F3
		RamWrite32A_Ofilm( mes2ae_OFILM, 0x00000000 ) ;		// 0x11F4
		RamWrite32A_Ofilm( mes2ba_OFILM, 0x3CA175C0 ) ;		// 0x11F5	LPF150Hz
		RamWrite32A_Ofilm( mes2bb_OFILM, 0x3CA175C0 ) ;		// 0x11F6
		RamWrite32A_Ofilm( mes2bc_OFILM, 0x3F75E8C0 ) ;		// 0x11F7
		RamWrite32A_Ofilm( mes2bd_OFILM, 0x00000000 ) ;		// 0x11F8
		RamWrite32A_Ofilm( mes2be_OFILM, 0x00000000 ) ;		// 0x11F9
	}
#endif
}



//********************************************************************************
// Function Name 	: SrvCon_OFILM
// Retun Value		: NON
// Argment Value	: X or Y Select, Servo ON_OFILM/OFF_OFILM
// Explanation		: Servo ON_OFILM,OFF_OFILM Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	SrvCon_OFILM( unsigned char	UcDirSel_OFILM, unsigned char	UcSwcCon_OFILM )
{
	if( UcSwcCon_OFILM ) {
		if( !UcDirSel_OFILM ) {						// X Direction
			RegWriteA_Ofilm( WH_EQSWX_OFILM , 0x03 ) ;			// 0x0170
			RamWrite32A_Ofilm( sxggf_OFILM, 0x00000000 ) ;		// 0x10B5
		} else {								// Y Direction
			RegWriteA_Ofilm( WH_EQSWY_OFILM , 0x03 ) ;			// 0x0171
			RamWrite32A_Ofilm( syggf_OFILM, 0x00000000 ) ;		// 0x11B5
		}
	} else {
		if( !UcDirSel_OFILM ) {						// X Direction
			RegWriteA_Ofilm( WH_EQSWX_OFILM , 0x02 ) ;			// 0x0170
			RamWrite32A_Ofilm( SXLMT_OFILM, 0x00000000 ) ;		// 0x1477
		} else {								// Y Direction
			RegWriteA_Ofilm( WH_EQSWY_OFILM , 0x02 ) ;			// 0x0171
			RamWrite32A_Ofilm( SYLMT_OFILM, 0x00000000 ) ;		// 0x14F7
		}
	}
}



#ifdef	MODULE_CALIBRATION_OFILM
//********************************************************************************
// Function Name 	: LopGan_OFILM
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
unsigned char	LopGan_OFILM( unsigned char	UcDirSel_OFILM )
{
	unsigned char	UcLpAdjSts_OFILM ;
	
 #ifdef	HALLADJ_HW_OFILM
	UcLpAdjSts_OFILM	= LoopGainAdj_OFILM( UcDirSel_OFILM ) ;
 #else
	MesFil_OFILM( LOOPGAIN_OFILM ) ;

	// Servo ON_OFILM
	SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;
	SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;

	// Wait 300ms
	WitTim_Ofilm( 300 ) ;

	// Loop Gain Adjust Initialize
	LopIni_OFILM( UcDirSel_OFILM ) ;

	// Loop Gain Adjust
	UcLpAdjSts_OFILM	= LopAdj_OFILM( UcDirSel_OFILM ) ;
 #endif
	// Servo OFF_OFILM
	SrvCon_OFILM( X_DIR_OFILM, OFF_OFILM ) ;
	SrvCon_OFILM( Y_DIR_OFILM, OFF_OFILM ) ;

	if( !UcLpAdjSts_OFILM ) {
		return( EXE_END_OFILM ) ;
	} else {
		if( !UcDirSel_OFILM ) {
			return( EXE_LXADJ_OFILM ) ;
		} else {
			return( EXE_LYADJ_OFILM ) ;
		}
	}
}



 #ifndef	HALLADJ_HW_OFILM
//********************************************************************************
// Function Name 	: LopIni_OFILM
// Retun Value		: NON
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Initialize Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopIni_OFILM( unsigned char	UcDirSel_OFILM )
{
	// Loop Gain Value Initialize
	LopPar_OFILM( UcDirSel_OFILM ) ;

	// Sign Wave Output Setting
	LopSin_OFILM( UcDirSel_OFILM, ON_OFILM ) ;

}
 #endif


//********************************************************************************
// Function Name 	: LopPar_OFILM
// Retun Value		: NON
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Parameter Initialize Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	LopPar_OFILM( unsigned char	UcDirSel_OFILM )
{
	unsigned short	UsLopGan_OFILM ;

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	if( !UcDirSel_OFILM ) {
		UsLopGan_OFILM	= SXGAIN_LOP_OFILM ;
		RamWriteA_Ofilm( sxg_OFILM, UsLopGan_OFILM ) ;			/* 0x10D3 */
	} else {
		UsLopGan_OFILM	= SYGAIN_LOP_OFILM ;
		RamWriteA_Ofilm( syg_OFILM, UsLopGan_OFILM ) ;			/* 0x11D3 */
	}

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
}



 #ifndef	HALLADJ_HW_OFILM
//********************************************************************************
// Function Name 	: LopSin_OFILM
// Retun Value		: NON
// Argment Value	: X,Y Direction, ON_OFILM/OFF_OFILM Switch
// Explanation		: Loop Gain Adjust Sign Wave Initialize Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopSin_OFILM( unsigned char	UcDirSel_OFILM, unsigned char	UcSonOff_OFILM )
{
	unsigned short		UsFreqVal_OFILM ;
	unsigned char		UcEqSwX_OFILM , UcEqSwY_OFILM ;
	
	RegReadA_Ofilm( WH_EQSWX_OFILM, &UcEqSwX_OFILM ) ;				/* 0x0170	*/
	RegReadA_Ofilm( WH_EQSWY_OFILM, &UcEqSwY_OFILM ) ;				/* 0x0171	*/
		
	if( UcSonOff_OFILM ) {
		
  #ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
		/* Freq = CmSinFrq * 11.718kHz / 65536 / 16 */
		UsFreqVal_OFILM	=	0x29F1 ;				/* 119.9Hz */
  #else
		/* Freq = CmSinFrq * 23.4375kHz / 65536 / 16 */
		UsFreqVal_OFILM	=	0x14F8 ;				/* 119.9Hz */
  #endif
		
		RegWriteA_Ofilm( WC_SINFRQ0_OFILM,	(unsigned char)UsFreqVal_OFILM ) ;				// 0x0181		Freq L
		RegWriteA_Ofilm( WC_SINFRQ1_OFILM,	(unsigned char)(UsFreqVal_OFILM >> 8) ) ;			// 0x0182		Freq H
		
		if( !UcDirSel_OFILM ) {

			UcEqSwX_OFILM |= 0x10 ;
			UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
			
			RamWrite32A_Ofilm( sxsin_OFILM, 0x3CA3D70A ) ;				// 0x10D5		-34dB
		} else {

			UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
			UcEqSwY_OFILM |= 0x10 ;
			
			RamWrite32A_Ofilm( sysin_OFILM, 0x3CA3D70A ) ;				// 0x11D5		-34dB
		}
		RegWriteA_Ofilm( WC_SINPHSX_OFILM, 0x00 ) ;					/* 0x0183	X Sine phase */
		RegWriteA_Ofilm( WC_SINPHSY_OFILM, 0x00 ) ;					/* 0x0184	Y Sine phase */
		RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	Switch control */
		RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	Switch control */
		RegWriteA_Ofilm( WC_SINON_OFILM,     0x01 ) ;				/* 0x0180	Sine wave  */
	} else {
		UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
		UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
		RegWriteA_Ofilm( WC_SINON_OFILM,     0x00 ) ;				/* 0x0180	Sine wave  */
		if( !UcDirSel_OFILM ) {
			RamWrite32A_Ofilm( sxsin_OFILM, 0x00000000 ) ;			// 0x10D5
		} else {
			RamWrite32A_Ofilm( sysin_OFILM, 0x00000000 ) ;			// 0x11D5
		}
		RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	Switch control */
		RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	Switch control */
	}
}



//********************************************************************************
// Function Name 	: LopAdj_OFILM
// Retun Value		: Command Status
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
unsigned char	LopAdj_OFILM( unsigned char	UcDirSel_OFILM )
{
#if 1
	return 0;
#else

	unsigned char	UcAdjSts_OFILM	= FAILURE_OFILM ;
	unsigned short	UsRtnVal_OFILM ;
	float			SfCmpVal_OFILM ;
	unsigned char	UcIdxCnt_OFILM ;
	unsigned char	UcIdxCn1_OFILM ;
	unsigned char	UcIdxCn2_OFILM ;
	UnFltVal_OFILM		UnAdcXg1_OFILM, UnAdcXg2_OFILM , UnRtnVa_OFILM ;

	float			DfGanVal_OFILM[ 5 ] ;
	float			DfTemVal_OFILM ;

	if( !UcDirSel_OFILM ) {
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM, (unsigned char)SXGZ_OFILM ) ;							// 0x0194
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM, (unsigned char)(( SXGZ_OFILM >> 8 ) & 0x0001 ) ) ;	// 0x0195
		RegWriteA_Ofilm( WC_MES2ADD0_OFILM, (unsigned char)SXG3Z ) ;						// 0x0196
		RegWriteA_Ofilm( WC_MES2ADD1_OFILM, (unsigned char)(( SXG3Z >> 8 ) & 0x0001 ) ) ;	// 0x0197
TRACE(" X axis " ) ;
	} else {
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM, (unsigned char)SYGZ ) ;							// 0x0194
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM, (unsigned char)(( SYGZ >> 8 ) & 0x0001 ) ) ;	// 0x0195
		RegWriteA_Ofilm( WC_MES2ADD0_OFILM, (unsigned char)SYG3Z ) ;						// 0x0196
		RegWriteA_Ofilm( WC_MES2ADD1_OFILM, (unsigned char)(( SYG3Z >> 8 ) & 0x0001 ) ) ;	// 0x0197
TRACE(" Y axis " ) ;
	}
	
	// 5 Times Average Value Calculation
	for( UcIdxCnt_OFILM = 0 ; UcIdxCnt_OFILM < 5 ; UcIdxCnt_OFILM++ )
	{
		LopMes_OFILM( ) ;																// Loop Gain Mesurement Start

		UnAdcXg1_OFILM.UlLngVal_OFILM	= GinMes_OFILM( MES_XG1 ) ;										// LXG1 Measure
		UnAdcXg2_OFILM.UlLngVal_OFILM	= GinMes_OFILM( MES_XG2 ) ;										// LXG2 Measure

TRACE("        DfAdcXg1 = %08xh ", (unsigned int)UnAdcXg1_OFILM.UlLngVal_OFILM ) ;
TRACE("   DfAdcXg2 = %08xh\n", (unsigned int)UnAdcXg2_OFILM.UlLngVal_OFILM ) ;
		SfCmpVal_OFILM	= (float)UnAdcXg2_OFILM.SfFltVal_OFILM / UnAdcXg1_OFILM.SfFltVal_OFILM ;					// Compare Coefficient Value

		if( !UcDirSel_OFILM ) {
			RamRead32A_Ofilm( sxg_OFILM, &UnRtnVa_OFILM.UlLngVal_OFILM ) ;									// 0x10D3
		} else {
			RamRead32A_Ofilm( syg_OFILM, &UnRtnVa_OFILM.UlLngVal_OFILM ) ;									// 0x11D3
		}
		UnRtnVa_OFILM.SfFltVal_OFILM	=  (float)UnRtnVa_OFILM.SfFltVal_OFILM * SfCmpVal_OFILM ;
TRACE("        DfRtnVal = %08xh\n", (unsigned int)UnRtnVa_OFILM.UlLngVal_OFILM ) ;

		DfGanVal_OFILM[ UcIdxCnt_OFILM ]	= (float)UnRtnVa_OFILM.SfFltVal_OFILM ;
	}

	for( UcIdxCn1_OFILM = 0 ; UcIdxCn1_OFILM < 4 ; UcIdxCn1_OFILM++ )
	{
		for( UcIdxCn2_OFILM = UcIdxCn1_OFILM+1 ; UcIdxCn2_OFILM < 5 ; UcIdxCn2_OFILM++ )
		{
			if( DfGanVal_OFILM[ UcIdxCn1_OFILM ] > (float)DfGanVal_OFILM[ UcIdxCn2_OFILM ] ) {
				DfTemVal_OFILM				= DfGanVal_OFILM[ UcIdxCn1_OFILM ] ;
				DfGanVal_OFILM[ UcIdxCn1_OFILM ]	= (float)DfGanVal_OFILM[ UcIdxCn2_OFILM ] ;
				DfGanVal_OFILM[ UcIdxCn2_OFILM ]	= (float)DfTemVal_OFILM ;
			}
		}
	}

	UnRtnVa_OFILM.SfFltVal_OFILM	= (float)( DfGanVal_OFILM[ 1 ] + DfGanVal_OFILM[ 2 ] + DfGanVal_OFILM[ 3 ] ) / 3  ;
TRACE("      LopGVal1 = %08xh ", (unsigned int)DfGanVal_OFILM[ 1 ] ) ;
TRACE("      LopGVal2 = %08xh ", (unsigned int)DfGanVal_OFILM[ 2 ] ) ;
TRACE("      LopGVal3 = %08xh \n", (unsigned int)DfGanVal_OFILM[ 3 ] ) ;
TRACE("      LopVal = %08xh \n", (unsigned int)UnRtnVa_OFILM.UlLngVal_OFILM ) ;

	LopSin_OFILM( UcDirSel_OFILM, OFF_OFILM ) ;

	if( UnRtnVa_OFILM.UlLngVal_OFILM < 0x3F800000 ) {									// Adjust Error
		UcAdjSts_OFILM	= SUCCESS_OFILM ;												// Status OK
	}

	if( UcAdjSts_OFILM ) {
		if( !UcDirSel_OFILM ) {
			RamWrite32A_Ofilm( sxg_OFILM, 0x3F800000 ) ;						// 0x10D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM	= 0x7FFF ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgSts_OFILM	= 0x0000 ;
		} else {
			RamWrite32A_Ofilm( syg_OFILM, 0x3F800000 ) ;						// 0x11D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM	= 0x7FFF ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLygSts_OFILM	= 0x0000 ;
		}
	} else {
		if( !UcDirSel_OFILM ) {
			RamWrite32A_Ofilm( sxg_OFILM, UnRtnVa_OFILM.UlLngVal_OFILM ) ;						// 0x10D3
			RamAccFixMod_OFILM( ON_OFILM ) ;								// Fix mode
			RamReadA_Ofilm( sxg_OFILM, &UsRtnVal_OFILM ) ;						// 0x10D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM	= UsRtnVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgSts_OFILM	= 0xFFFF ;
		} else {
			RamWrite32A_Ofilm( syg_OFILM, UnRtnVa_OFILM.UlLngVal_OFILM ) ;						// 0x11D3
			RamAccFixMod_OFILM( ON_OFILM ) ;								// Fix mode
			RamReadA_Ofilm( syg_OFILM, &UsRtnVal_OFILM ) ;						// 0x11D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM	= UsRtnVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLygSts_OFILM	= 0xFFFF ;
		}
TRACE("      16BIT = %04xh \n", UsRtnVal_OFILM ) ;
		RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	}
	return( UcAdjSts_OFILM ) ;
	#endif
}


//********************************************************************************
// Function Name 	: LopMes_OFILM
// Retun Value		: void
// Argment Value	: void
// Explanation		: Loop Gain Adjust Measure Setting
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	LopMes_OFILM( void )
{
	ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );					// Measure Filter RAM Clear
	RamWrite32A_Ofilm( MSABS1AV_OFILM, 0x00000000 ) ;			// 0x1041	Clear
	RamWrite32A_Ofilm( MSABS2AV_OFILM, 0x00000000 ) ;			// 0x1141	Clear
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x04 ) ;				// 0x0193
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x00 ) ;				// 0x0192	1024 Times Measure
	RamWrite32A_Ofilm( msmean_OFILM	, 0x3A800000 );				// 0x1230	1/CmMesLoop[15:0]
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x01 ) ;					// 0x0198	ABS
	RegWriteA_Ofilm( WC_MESWAIT_OFILM,     0x00 ) ;				/* 0x0199	0 cross wait */
	BsyWit_OFILM( WC_MESMODE_OFILM, 0x01 ) ;					// 0x0190	Sin Wave Measure
}


//********************************************************************************
// Function Name 	: GinMes_OFILM
// Retun Value		: Measure Result
// Argment Value	: MES1/MES2 Select
// Explanation		: Measure Result Read
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
unsigned long	GinMes_OFILM( unsigned char	UcXg1Xg2_OFILM )
{
	unsigned long	UlMesVal_OFILM ;

	if( !UcXg1Xg2_OFILM ) {
		RamRead32A_Ofilm( MSABS1AV_OFILM, &UlMesVal_OFILM ) ;			// 0x1041
	} else {
		RamRead32A_Ofilm( MSABS2AV_OFILM, &UlMesVal_OFILM ) ;			// 0x1141
	}
	
	return( UlMesVal_OFILM ) ;
}

 #endif
#endif

//********************************************************************************
// Function Name 	: TneGvc_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Gyro VC offset
// History			: First edition 						2013.01.15  Y.Shigeoka
//********************************************************************************
#define	LIMITH_OFILM		0x0FA0
#define	LIMITL_OFILM		0xF060
#define	INITVAL_OFILM		0x0000
unsigned char	TneGvc_OFILM( void )
{
	unsigned char  UcRsltSts_OFILM;
	
	
	// A/D Offset Clear
	RegWriteA_Ofilm( IZAH_OFILM,	(unsigned char)(INITVAL_OFILM >> 8) ) ;	// 0x02A0		Set Offset High byte
	RegWriteA_Ofilm( IZAL_OFILM,	(unsigned char)INITVAL_OFILM ) ;			// 0x02A1		Set Offset Low byte
	RegWriteA_Ofilm( IZBH_OFILM,	(unsigned char)(INITVAL_OFILM >> 8) ) ;	// 0x02A2		Set Offset High byte
	RegWriteA_Ofilm( IZBL_OFILM,	(unsigned char)INITVAL_OFILM ) ;			// 0x02A3		Set Offset Low byte
	
TRACE("FNC=TneGvc_OFILM\n");
	MesFil_OFILM( THROUGH_OFILM ) ;				// \91\AA\92\E8\97p\83t\83B\83\8B\83^\81[\82\F0\90ݒ肷\82\E9\81B
	//////////
	// X
	//////////
	RegWriteA_Ofilm( WC_MES1ADD0_OFILM, 0x00 ) ;		// 0x0194
	RegWriteA_Ofilm( WC_MES1ADD1_OFILM, 0x00 ) ;		// 0x0195
	ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );					// Measure Filter RAM Clear
	StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM = (unsigned short)GenMes_OFILM( AD2Z_OFILM, 0 );		// 64\89\F1\82̕\BD\8Bϒl\91\AA\92\E8	GYRMON1(0x1110) <- GXADZ(0x144A)
	RegWriteA_Ofilm( IZAH_OFILM, (unsigned char)(StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM >> 8) ) ;	// 0x02A0		Set Offset High byte
	RegWriteA_Ofilm( IZAL_OFILM, (unsigned char)(StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM) ) ;		// 0x02A1		Set Offset Low byte
TRACE( " Meas  A = %04xh\n", StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM );
	//////////
	// Y
	//////////
	RegWriteA_Ofilm( WC_MES1ADD0_OFILM, 0x00 ) ;		// 0x0194
	RegWriteA_Ofilm( WC_MES1ADD1_OFILM, 0x00 ) ;		// 0x0195
	ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );					// Measure Filter RAM Clear
	StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM = (unsigned short)GenMes_OFILM( AD3Z_OFILM, 0 );		// 64\89\F1\82̕\BD\8Bϒl\91\AA\92\E8	GYRMON2(0x1111) <- GYADZ(0x14CA)
	RegWriteA_Ofilm( IZBH_OFILM, (unsigned char)(StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM >> 8) ) ;	// 0x02A2		Set Offset High byte
	RegWriteA_Ofilm( IZBL_OFILM, (unsigned char)(StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM) ) ;		// 0x02A3		Set Offset Low byte
TRACE( " Meas  B = %04xh\n", StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM );
	
	UcRsltSts_OFILM = EXE_END_OFILM ;						/* Clear Status */

	StAdjPar_OFILM.StGvcOff_OFILM.UsGxoSts_OFILM	= 0xFFFF ;
	if(( (short)StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM < (short)LIMITL_OFILM ) || ( (short)StAdjPar_OFILM.StGvcOff_OFILM.UsGxoVal_OFILM > (short)LIMITH_OFILM ))
	{
		UcRsltSts_OFILM |= EXE_GXADJ_OFILM ;
		StAdjPar_OFILM.StGvcOff_OFILM.UsGxoSts_OFILM	= 0x0000 ;
TRACE("    Gyro X1 Error \n" );
	}
	
	StAdjPar_OFILM.StGvcOff_OFILM.UsGyoSts_OFILM	= 0xFFFF ;
	if(( (short)StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM < (short)LIMITL_OFILM ) || ( (short)StAdjPar_OFILM.StGvcOff_OFILM.UsGyoVal_OFILM > (short)LIMITH_OFILM ))
	{
		UcRsltSts_OFILM |= EXE_GYADJ_OFILM ;
		StAdjPar_OFILM.StGvcOff_OFILM.UsGyoSts_OFILM	= 0x0000 ;
TRACE("    Gyro Y1 Error \n" );
	}
	return( UcRsltSts_OFILM );
		
}



//********************************************************************************
// Function Name 	: RtnCen_OFILM
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
unsigned char	RtnCen_OFILM( unsigned char	UcCmdPar_OFILM )
{
	unsigned char	UcCmdSts_OFILM ;

	UcCmdSts_OFILM	= EXE_END_OFILM ;

	GyrCon_OFILM( OFF_OFILM ) ;											// Gyro OFF_OFILM

	if( !UcCmdPar_OFILM ) {										// X,Y Centering

		StbOnn_OFILM() ;											// Slope Mode
		
	} else if( UcCmdPar_OFILM == 0x01 ) {							// X Centering Only

		SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;								// X only Servo ON_OFILM
		SrvCon_OFILM( Y_DIR_OFILM, OFF_OFILM ) ;
	} else if( UcCmdPar_OFILM == 0x02 ) {							// Y Centering Only

		SrvCon_OFILM( X_DIR_OFILM, OFF_OFILM ) ;								// Y only Servo ON_OFILM
		SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;
	}

	return( UcCmdSts_OFILM ) ;
}



//********************************************************************************
// Function Name 	: GyrCon_OFILM
// Retun Value		: NON
// Argment Value	: Gyro Filter ON_OFILM or OFF_OFILM
// Explanation		: Gyro Filter Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	GyrCon_OFILM( unsigned char	UcGyrCon_OFILM )
{
	// Return HPF Setting
	RegWriteA_Ofilm( WG_SHTON_OFILM, 0x00 ) ;									// 0x0107
	
	if( UcGyrCon_OFILM == ON_OFILM ) {												// Gyro ON_OFILM

		
#ifdef	GAIN_CONT_OFILM
		/* Gain3 Register */
//		AutoGainControlSw_OFILM( ON_OFILM ) ;											/* Auto Gain Control Mode ON_OFILM */
#endif
		ClrGyr_OFILM( 0x000E , CLR_FRAM1_OFILM );		// Gyro Delay RAM Clear

		RamWrite32A_Ofilm( sxggf_OFILM, 0x3F800000 ) ;	// 0x10B5
		RamWrite32A_Ofilm( syggf_OFILM, 0x3F800000 ) ;	// 0x11B5
		
	} else if( UcGyrCon_OFILM == SPC_OFILM ) {										// Gyro ON_OFILM for LINE

		
#ifdef	GAIN_CONT_OFILM
		/* Gain3 Register */
//		AutoGainControlSw_OFILM( ON_OFILM ) ;											/* Auto Gain Control Mode ON_OFILM */
#endif

		RamWrite32A_Ofilm( sxggf_OFILM, 0x3F800000 ) ;	// 0x10B5
		RamWrite32A_Ofilm( syggf_OFILM, 0x3F800000 ) ;	// 0x11B5
		

	} else {															// Gyro OFF_OFILM
		
		RamWrite32A_Ofilm( sxggf_OFILM, 0x00000000 ) ;	// 0x10B5
		RamWrite32A_Ofilm( syggf_OFILM, 0x00000000 ) ;	// 0x11B5
		

#ifdef	GAIN_CONT_OFILM
		/* Gain3 Register */
//		AutoGainControlSw_OFILM( OFF_OFILM ) ;											/* Auto Gain Control Mode OFF_OFILM */
#endif
	}
}



//********************************************************************************
// Function Name 	: OisEna_OFILM
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	OisEna_OFILM( void )
{
	// Servo ON_OFILM
	SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;
	SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;

	GyrCon_OFILM( ON_OFILM ) ;
}

//********************************************************************************
// Function Name 	: OisEnaLin_OFILM
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function for Line adjustment
// History			: First edition 						2013.09.05 Y.Shigeoka
//********************************************************************************
void	OisEnaLin_OFILM( void )
{
	// Servo ON_OFILM
	SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;
	SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;

	GyrCon_OFILM( SPC_OFILM ) ;
}



//********************************************************************************
// Function Name 	: TimPro_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Timer Interrupt Process Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	TimPro_OFILM( void )
{
#ifdef	MODULE_CALIBRATION_OFILM
	if( UcOscAdjFlg_OFILM )
	{
		if( UcOscAdjFlg_OFILM == MEASSTR_OFILM )
		{
			RegWriteA_Ofilm( OSCCNTEN_OFILM, 0x01 ) ;		// 0x0258	OSC Cnt enable
			UcOscAdjFlg_OFILM = MEASCNT_OFILM ;
		}
		else if( UcOscAdjFlg_OFILM == MEASCNT_OFILM )
		{
			RegWriteA_Ofilm( OSCCNTEN_OFILM, 0x00 ) ;		// 0x0258	OSC Cnt disable
			UcOscAdjFlg_OFILM = MEASFIX_OFILM ;
		}
	}
#endif
}



//********************************************************************************
// Function Name 	: S2cPro_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: S2 Command Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	S2cPro_OFILM( unsigned char uc_mode_OFILM )
{
	if( uc_mode_OFILM == 1 )
	{
#ifdef H1COEF_CHANGER_OFILM
		SetH1cMod_OFILM( S2MODE_OFILM ) ;							/* cancel Lvl change */
#endif
		// HPF\81\A8Through Setting
		RegWriteA_Ofilm( WG_SHTON_OFILM, 0x11 ) ;							// 0x0107
		RamWrite32A_Ofilm( gxh1c_OFILM, DIFIL_S2_OFILM );							// 0x1012
		RamWrite32A_Ofilm( gyh1c_OFILM, DIFIL_S2_OFILM );							// 0x1112
TRACE(" Limit3 = (S2 On) \n" ) ;
	}
	else
	{
		RamWrite32A_Ofilm( gxh1c_OFILM, UlH1Coefval_OFILM );							// 0x1012
		RamWrite32A_Ofilm( gyh1c_OFILM, UlH1Coefval_OFILM );							// 0x1112
		// HPF\81\A8Through Setting
		RegWriteA_Ofilm( WG_SHTON_OFILM, 0x00 ) ;							// 0x0107

#ifdef H1COEF_CHANGER_OFILM
		SetH1cMod_OFILM( UcH1LvlMod_OFILM ) ;							/* Re-setting */
#endif
TRACE(" Limit3 = (S2 Off)\n" ) ;
	}
	
}


//********************************************************************************
// Function Name 	: GenMes_OFILM
// Retun Value		: A/D Convert Result
// Argment Value	: Measure Filter Input Signal Ram Address
// Explanation		: General Measure Function
// History			: First edition 						2013.01.10 Y.Shigeoka
//********************************************************************************
short	GenMes_OFILM( unsigned short	UsRamAdd_OFILM, unsigned char	UcMesMod_OFILM )
{
	short	SsMesRlt_OFILM ;

	RegWriteA_Ofilm( WC_MES1ADD0_OFILM, (unsigned char)UsRamAdd_OFILM ) ;							// 0x0194
	RegWriteA_Ofilm( WC_MES1ADD1_OFILM, (unsigned char)(( UsRamAdd_OFILM >> 8 ) & 0x0001 ) ) ;	// 0x0195
	RamWrite32A_Ofilm( MSABS1AV_OFILM, 0x00000000 ) ;				// 0x1041	Clear
	
	if( !UcMesMod_OFILM ) {
		RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x04 ) ;				// 0x0193
		RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x00 ) ;				// 0x0192	1024 Times Measure
		RamWrite32A_Ofilm( msmean_OFILM	, 0x3A7FFFF7 );				// 0x1230	1/CmMesLoop[15:0]
	} else {
		RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x00 ) ;				// 0x0193
		RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x01 ) ;				// 0x0192	1 Times Measure
		RamWrite32A_Ofilm( msmean_OFILM	, 0x3F800000 );				// 0x1230	1/CmMesLoop[15:0]
	}

	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x00 ) ;						// 0x0198	none ABS
	BsyWit_OFILM( WC_MESMODE_OFILM, 0x01 ) ;						// 0x0190	normal Measure

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	RamReadA_Ofilm( MSABS1AV_OFILM, ( unsigned short * )&SsMesRlt_OFILM ) ;	// 0x1041

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
	return( SsMesRlt_OFILM ) ;
}


//********************************************************************************
// Function Name 	: SetSinWavePara_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
#ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
	/********* Parameter Setting *********/
	/* Servo Sampling Clock		=	11.71875kHz						*/
	/* Freq						=	CmSinFreq*Fs/65536/16			*/
	/* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
const unsigned short	CucFreqVal_OFILM[ 17 ]	= {
		0xFFFF,				//  0:  Stop
		0x0059,				//  1: 0.994653Hz
		0x00B2,				//  2: 1.989305Hz
		0x010C,				//  3: 2.995133Hz	
		0x0165,				//  4: 3.989786Hz
		0x01BF,				//  5: 4.995614Hz
		0x0218,				//  6: 5.990267Hz
		0x0272,				//  7: 6.996095Hz
		0x02CB,				//  8: 7.990748Hz
		0x0325,				//  9: 8.996576Hz
		0x037E,				//  A: 9.991229Hz
		0x03D8,				//  B: 10.99706Hz
		0x0431,				//  C: 11.99171Hz
		0x048B,				//  D: 12.99754Hz
		0x04E4,				//  E: 13.99219Hz
		0x053E,				//  F: 14.99802Hz
		0x0597				// 10: 15.99267Hz
	} ;
#else
	/********* Parameter Setting *********/
	/* Servo Sampling Clock		=	23.4375kHz						*/
	/* Freq						=	CmSinFreq*Fs/65536/16			*/
	/* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
const unsigned short	CucFreqVal_OFILM[ 17 ]	= {
		0xFFFF,				//  0:  Stop
		0x002C,				//  1: 0.983477Hz
		0x0059,				//  2: 1.989305Hz
		0x0086,				//  3: 2.995133Hz	
		0x00B2,				//  4: 3.97861Hz
		0x00DF,				//  5: 4.984438Hz
		0x010C,				//  6: 5.990267Hz
		0x0139,				//  7: 6.996095Hz
		0x0165,				//  8: 7.979572Hz
		0x0192,				//  9: 8.9854Hz
		0x01BF,				//  A: 9.991229Hz
		0x01EC,				//  B: 10.99706Hz
		0x0218,				//  C: 11.98053Hz
		0x0245,				//  D: 12.98636Hz
		0x0272,				//  E: 13.99219Hz
		0x029F,				//  F: 14.99802Hz
		0x02CB				// 10: 15.9815Hz
	} ;
#endif
	
#define		USE_SINLPF_OFILM			/* if sin or circle movement is used LPF , this define has to enable */
	
/* \90U\95\9D\82\CDsxsin(0x10D5),sysin_OFILM(0x11D5)\82Œ\B2\90\AE */
void	SetSinWavePara_OFILM( unsigned char UcTableVal_OFILM ,  unsigned char UcMethodVal_OFILM )
{
	unsigned short	UsFreqDat_OFILM ;
	unsigned char	UcEqSwX_OFILM , UcEqSwY_OFILM ;

	
	if(UcTableVal_OFILM > 0x10 )
		UcTableVal_OFILM = 0x10 ;			/* Limit */
	UsFreqDat_OFILM = CucFreqVal_OFILM[ UcTableVal_OFILM ] ;	
	
	if( UcMethodVal_OFILM == SINEWAVE_OFILM) {
		RegWriteA_Ofilm( WC_SINPHSX_OFILM, 0x00 ) ;					/* 0x0183	*/
		RegWriteA_Ofilm( WC_SINPHSY_OFILM, 0x00 ) ;					/* 0x0184	*/
	}else if( UcMethodVal_OFILM == CIRCWAVE_OFILM ){
		RegWriteA_Ofilm( WC_SINPHSX_OFILM,	0x00 ) ;				/* 0x0183	*/
		RegWriteA_Ofilm( WC_SINPHSY_OFILM,	0x20 ) ;				/* 0x0184	*/
	}else{
		RegWriteA_Ofilm( WC_SINPHSX_OFILM, 0x00 ) ;					/* 0x0183	*/
		RegWriteA_Ofilm( WC_SINPHSY_OFILM, 0x00 ) ;					/* 0x0184	*/
	}

#ifdef	USE_SINLPF_OFILM
	if(( UcMethodVal_OFILM == CIRCWAVE_OFILM ) || ( UcMethodVal_OFILM == SINEWAVE_OFILM )) {
		MesFil_OFILM( NOISE_OFILM ) ;			/* LPF */
	}
#endif

	if( UsFreqDat_OFILM == 0xFFFF )			/* Sine\94g\92\86\8E~ */
	{

		RegReadA_Ofilm( WH_EQSWX_OFILM, &UcEqSwX_OFILM ) ;				/* 0x0170	*/
		RegReadA_Ofilm( WH_EQSWY_OFILM, &UcEqSwY_OFILM ) ;				/* 0x0171	*/
		UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
		UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
		RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	*/
		RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	*/
		
#ifdef	USE_SINLPF_OFILM
		if(( UcMethodVal_OFILM == CIRCWAVE_OFILM ) || ( UcMethodVal_OFILM == SINEWAVE_OFILM ) || ( UcMethodVal_OFILM == XACTTEST_OFILM ) || ( UcMethodVal_OFILM == YACTTEST_OFILM )) {
			RegWriteA_Ofilm( WC_DPON_OFILM,     0x00 ) ;			/* 0x0105	Data pass off */
			RegWriteA_Ofilm( WC_DPO1ADD0_OFILM, 0x00 ) ;			/* 0x01B8	output initial */
			RegWriteA_Ofilm( WC_DPO1ADD1_OFILM, 0x00 ) ;			/* 0x01B9	output initial */
			RegWriteA_Ofilm( WC_DPO2ADD0_OFILM, 0x00 ) ;			/* 0x01BA	output initial */
			RegWriteA_Ofilm( WC_DPO2ADD1_OFILM, 0x00 ) ;			/* 0x01BB	output initial */
			RegWriteA_Ofilm( WC_DPI1ADD0_OFILM, 0x00 ) ;			/* 0x01B0	input initial */
			RegWriteA_Ofilm( WC_DPI1ADD1_OFILM, 0x00 ) ;			/* 0x01B1	input initial */
			RegWriteA_Ofilm( WC_DPI2ADD0_OFILM, 0x00 ) ;			/* 0x01B2	input initial */
			RegWriteA_Ofilm( WC_DPI2ADD1_OFILM, 0x00 ) ;			/* 0x01B3	input initial */
			
			/* Ram Access */
			RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
			
			RamWriteA_Ofilm( SXOFFZ1_OFILM, UsCntXof_OFILM ) ;			/* 0x1461	set optical value */
			RamWriteA_Ofilm( SYOFFZ1_OFILM, UsCntYof_OFILM ) ;			/* 0x14E1	set optical value */
			
			/* Ram Access */
			RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
			RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  0x00 ) ;			/* 0x0194	*/
			RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  0x00 ) ;			/* 0x0195	*/
			RegWriteA_Ofilm( WC_MES2ADD0_OFILM,  0x00 ) ;			/* 0x0196	*/
			RegWriteA_Ofilm( WC_MES2ADD1_OFILM,  0x00 ) ;			/* 0x0197	*/
			
		}
#endif
		RegWriteA_Ofilm( WC_SINON_OFILM,     0x00 ) ;			/* 0x0180	Sine wave  */
		
	}
	else
	{
		
		RegReadA_Ofilm( WH_EQSWX_OFILM, &UcEqSwX_OFILM ) ;				/* 0x0170	*/
		RegReadA_Ofilm( WH_EQSWY_OFILM, &UcEqSwY_OFILM ) ;				/* 0x0171	*/
		
		if(( UcMethodVal_OFILM == CIRCWAVE_OFILM ) || ( UcMethodVal_OFILM == SINEWAVE_OFILM )) {
#ifdef	USE_SINLPF_OFILM
			RegWriteA_Ofilm( WC_DPI1ADD0_OFILM,  ( unsigned char )MES1BZ2_OFILM ) ;						/* 0x01B0	input Meas-Fil */
			RegWriteA_Ofilm( WC_DPI1ADD1_OFILM,  ( unsigned char )(( MES1BZ2_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01B1	input Meas-Fil */
			RegWriteA_Ofilm( WC_DPI2ADD0_OFILM,  ( unsigned char )MES2BZ2_OFILM ) ;						/* 0x01B2	input Meas-Fil */
			RegWriteA_Ofilm( WC_DPI2ADD1_OFILM,  ( unsigned char )(( MES2BZ2_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01B3	input Meas-Fil */
			RegWriteA_Ofilm( WC_DPO1ADD0_OFILM, ( unsigned char )SXOFFZ1_OFILM ) ;						/* 0x01B8	output SXOFFZ1_OFILM */
			RegWriteA_Ofilm( WC_DPO1ADD1_OFILM, ( unsigned char )(( SXOFFZ1_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01B9	output SXOFFZ1_OFILM */
			RegWriteA_Ofilm( WC_DPO2ADD0_OFILM, ( unsigned char )SYOFFZ1_OFILM ) ;						/* 0x01BA	output SYOFFZ1_OFILM */
			RegWriteA_Ofilm( WC_DPO2ADD1_OFILM, ( unsigned char )(( SYOFFZ1_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01BA	output SYOFFZ1_OFILM */
			
			RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )SINXZ_OFILM ) ;							/* 0x0194	*/
			RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( SINXZ_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x0195	*/
			RegWriteA_Ofilm( WC_MES2ADD0_OFILM,  ( unsigned char )SINYZ_OFILM ) ;							/* 0x0196	*/
			RegWriteA_Ofilm( WC_MES2ADD1_OFILM,  ( unsigned char )(( SINYZ_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/
			
			RegWriteA_Ofilm( WC_DPON_OFILM,     0x03 ) ;			/* 0x0105	Data pass[1:0] on */
			
			UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
			UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
#else
			UcEqSwX_OFILM |= 0x08 ;
			UcEqSwY_OFILM |= 0x08 ;
#endif
		} else if(( UcMethodVal_OFILM == XACTTEST_OFILM ) || ( UcMethodVal_OFILM == YACTTEST_OFILM )) {
			RegWriteA_Ofilm( WC_DPI2ADD0_OFILM,  ( unsigned char )MES2BZ2_OFILM ) ;						/* 0x01B2	input Meas-Fil */
			RegWriteA_Ofilm( WC_DPI2ADD1_OFILM,  ( unsigned char )(( MES2BZ2_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01B3	input Meas-Fil */
			if( UcMethodVal_OFILM == XACTTEST_OFILM ){
				RegWriteA_Ofilm( WC_DPO2ADD0_OFILM, ( unsigned char )SXOFFZ1_OFILM ) ;						/* 0x01BA	output SXOFFZ1_OFILM */
				RegWriteA_Ofilm( WC_DPO2ADD1_OFILM, ( unsigned char )(( SXOFFZ1_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01BB	output SXOFFZ1_OFILM */
				RegWriteA_Ofilm( WC_MES2ADD0_OFILM,  ( unsigned char )SINXZ_OFILM ) ;							/* 0x0196	*/
				RegWriteA_Ofilm( WC_MES2ADD1_OFILM,  ( unsigned char )(( SINXZ_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/
			} else {
				RegWriteA_Ofilm( WC_DPO2ADD0_OFILM, ( unsigned char )SYOFFZ1_OFILM ) ;						/* 0x01BA	output SYOFFZ1_OFILM */
				RegWriteA_Ofilm( WC_DPO2ADD1_OFILM, ( unsigned char )(( SYOFFZ1_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x01BB	output SYOFFZ1_OFILM */
				RegWriteA_Ofilm( WC_MES2ADD0_OFILM,  ( unsigned char )SINYZ_OFILM ) ;							/* 0x0196	*/
				RegWriteA_Ofilm( WC_MES2ADD1_OFILM,  ( unsigned char )(( SINYZ_OFILM >> 8 ) & 0x0001 ) ) ;	/* 0x0197	*/
			}
			
			RegWriteA_Ofilm( WC_DPON_OFILM,     0x02 ) ;			/* 0x0105	Data pass[1] on */
			
			UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
			UcEqSwY_OFILM &= ~EQSINSW_OFILM ;

		}else{
			if( UcMethodVal_OFILM == XHALWAVE_OFILM ){
		    	UcEqSwX_OFILM = 0x22 ;				/* SW[5] */
//		    	UcEqSwY_OFILM = 0x03 ;
			}else{
//				UcEqSwX_OFILM = 0x03 ;
				UcEqSwY_OFILM = 0x22 ;				/* SW[5] */
			}
		}
		
		RegWriteA_Ofilm( WC_SINFRQ0_OFILM,	(unsigned char)UsFreqDat_OFILM ) ;				// 0x0181		Freq L
		RegWriteA_Ofilm( WC_SINFRQ1_OFILM,	(unsigned char)(UsFreqDat_OFILM >> 8) ) ;			// 0x0182		Freq H
		RegWriteA_Ofilm( WC_MESSINMODE_OFILM,     0x00 ) ;			/* 0x0191	Sine 0 cross  */

		RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	*/
		RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	*/

		RegWriteA_Ofilm( WC_SINON_OFILM,     0x01 ) ;			/* 0x0180	Sine wave  */
		
	}
	
	
}




#ifdef STANDBY_MODE_OFILM
//********************************************************************************
// Function Name 	: SetStandby_OFILM
// Retun Value		: NON
// Argment Value	: 0:Standby ON_OFILM 1:Standby OFF_OFILM 2:Standby2 ON_OFILM 3:Standby2 OFF_OFILM 
//					: 4:Standby3 ON_OFILM 5:Standby3 OFF_OFILM
// Explanation		: Set Standby
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	SetStandby_OFILM( unsigned char UcContMode_OFILM )
{
	unsigned char	UcStbb0_OFILM , UcClkon_OFILM ;
	
	switch(UcContMode_OFILM)
	{
	case STB1_ON_OFILM:

TRACE( " STB MODE = Stb1 ON_OFILM \n" ) ;
#ifdef	AF_PWMMODE
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#endif
		RegWriteA_Ofilm( STBB0_OFILM 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( STBB1_OFILM 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( PWMA_OFILM 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( CVA_OFILM,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw_OFILM( OFF_OFILM ) ;						/* Driver OFF_OFILM */
		AfDrvSw_OFILM( OFF_OFILM ) ;					/* AF Driver OFF_OFILM */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
//		RegWriteA_Ofilm( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep_OFILM( ON_OFILM ) ;				/* Gyro Sleep */
		break ;
	case STB1_OFF_OFILM:
		SelectGySleep_OFILM( OFF_OFILM ) ;				/* Gyro Wake Up */
//		RegWriteA_Ofilm( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw_OFILM( ON_OFILM ) ;						/* Driver Mode setting */
		AfDrvSw_OFILM( ON_OFILM ) ;						/* AF Driver Mode setting */
		RegWriteA_Ofilm( CVA_OFILM		, 0xC0 );		// 0x0020	Linear PWM mode enable
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( PWMA_OFILM		, 0xC0 );		// 0x0010	PWM enable
		RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( STBB0_OFILM	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb1 Off \n" ) ;
		break ;
	case STB2_ON_OFILM:
TRACE( " STB MODE = Stb2 ON_OFILM \n" ) ;
#ifdef	AF_PWMMODE
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#endif
		RegWriteA_Ofilm( STBB0_OFILM 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( STBB1_OFILM 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( PWMA_OFILM 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( CVA_OFILM,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw_OFILM( OFF_OFILM ) ;						/* Drvier Block Ena=0 */
		AfDrvSw_OFILM( OFF_OFILM ) ;					/* AF Drvier Block Ena=0 */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
//		RegWriteA_Ofilm( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep_OFILM( ON_OFILM ) ;				/* Gyro Sleep */
		RegWriteA_Ofilm( CLKON_OFILM, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF_OFILM + D-Gyro I/F OFF_OFILM	*/
		break ;
	case STB2_OFF_OFILM:
		RegWriteA_Ofilm( CLKON_OFILM,	0x1F ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep_OFILM( OFF_OFILM ) ;				/* Gyro Wake Up */
//		RegWriteA_Ofilm( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw_OFILM( ON_OFILM ) ;						/* Driver Mode setting */
		AfDrvSw_OFILM( ON_OFILM ) ;						/* AF Driver Mode setting */
		RegWriteA_Ofilm( CVA_OFILM, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( PWMA_OFILM	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( STBB0_OFILM	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb2 Off \n" ) ;
		break ;
	case STB3_ON_OFILM:
TRACE( " STB MODE = Stb3 ON_OFILM \n" ) ;
#ifdef	AF_PWMMODE
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#endif
		RegWriteA_Ofilm( STBB0_OFILM 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( STBB1_OFILM 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( PWMA_OFILM 	, 0x00 );			// 0x0010		PWM Standby
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( CVA_OFILM,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw_OFILM( OFF_OFILM ) ;						/* Drvier Block Ena=0 */
		AfDrvSw_OFILM( OFF_OFILM ) ;					/* AF Drvier Block Ena=0 */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
//		RegWriteA_Ofilm( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep_OFILM( ON_OFILM ) ;				/* Gyro Sleep */
		RegWriteA_Ofilm( CLKON_OFILM, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF_OFILM + D-Gyro I/F OFF_OFILM	*/
		RegWriteA_Ofilm( I2CSEL_OFILM, 0x01 ) ;			/* 0x0248	I2C Noise Cancel circuit OFF_OFILM	*/
		RegWriteA_Ofilm( OSCSTOP_OFILM, 0x02 ) ;		// 0x0256	Source Clock Input OFF_OFILM
		break ;
	case STB3_OFF_OFILM:
		RegWriteA_Ofilm( OSCSTOP_OFILM, 0x00 ) ;		// 0x0256	Source Clock Input ON_OFILM
		RegWriteA_Ofilm( I2CSEL_OFILM, 0x00 ) ;			/* 0x0248	I2C Noise Cancel circuit ON_OFILM	*/
		RegWriteA_Ofilm( CLKON_OFILM,	0x1F ) ;		// 0x020B	[ - | - | - | - | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep_OFILM( OFF_OFILM ) ;				/* Gyro Wake Up */
//		RegWriteA_Ofilm( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw_OFILM( ON_OFILM ) ;						/* Driver Mode setting */
		AfDrvSw_OFILM( ON_OFILM ) ;						/* AF Driver Mode setting */
		RegWriteA_Ofilm( CVA_OFILM, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA_Ofilm( PWMAAF_OFILM,	0x00 );			// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( PWMA_OFILM	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( STBB0_OFILM	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb3 Off \n" ) ;
		break ;
		
	case STB4_ON_OFILM:
TRACE( " STB MODE = Stb4 ON_OFILM \n" ) ;
#ifdef	AF_PWMMODE
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#endif
		RegWriteA_Ofilm( STBB0_OFILM 	, 0x00 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( STBB1_OFILM 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( PWMA_OFILM 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( CVA_OFILM,  	0x00 ) ;		/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw_OFILM( OFF_OFILM ) ;						/* Drvier Block Ena=0 */
		AfDrvSw_OFILM( OFF_OFILM ) ;					/* AF Drvier Block Ena=0 */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
//		RegWriteA_Ofilm( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		GyOutSignalCont_OFILM( ) ;				/* Gyro Continuos mode */
		RegWriteA_Ofilm( CLKON_OFILM, 0x04 ) ;			/* 0x020B	Servo & PWM Clock OFF_OFILM + D-Gyro I/F ON_OFILM	*/
		break ;
	case STB4_OFF_OFILM:
		RegWriteA_Ofilm( CLKON_OFILM,	0x1F ) ;		// 0x020B	[ - | - | - | - | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep_OFILM( OFF_OFILM ) ;				/* Gyro OIS mode */
//		RegWriteA_Ofilm( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw_OFILM( ON_OFILM ) ;						/* Driver Mode setting */
		AfDrvSw_OFILM( ON_OFILM ) ;						/* AF Driver Mode setting */
		RegWriteA_Ofilm( CVA_OFILM, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA_Ofilm( PWMAAF_OFILM, 	0x00 );			// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( PWMA_OFILM	, 	0xC0 );			// 0x0010	PWM enable
		RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( STBB0_OFILM	, 0xDF );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb4 Off \n" ) ;
		break ;
		
		/************** special mode ************/
	case STB2_OISON_OFILM:
TRACE( " STB MODE = Stb2 OISON \n" ) ;
		RegReadA_Ofilm( STBB0_OFILM 	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0_OFILM &= 0x80 ;
		RegWriteA_Ofilm( STBB0_OFILM 	, UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( PWMA_OFILM 	, 0x00 );		// 0x0010		PWM Standby
		RegWriteA_Ofilm( CVA_OFILM,  0x00 ) ;			/* 0x0020	LINEAR PWM mode standby	*/
		DrvSw_OFILM( OFF_OFILM ) ;						/* Drvier Block Ena=0 */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
//		RegWriteA_Ofilm( DACMONFC, 0x01 ) ;		// 0x0032	DAC Monitor Standby
		SelectGySleep_OFILM( ON_OFILM ) ;				/* Gyro Sleep */
		RegReadA_Ofilm( CLKON_OFILM, &UcClkon_OFILM ) ;		/* 0x020B	PWM Clock OFF_OFILM + D-Gyro I/F OFF_OFILM	SRVCLK can't OFF_OFILM */
		UcClkon_OFILM &= 0x1A ;
		RegWriteA_Ofilm( CLKON_OFILM, UcClkon_OFILM ) ;		/* 0x020B	PWM Clock OFF_OFILM + D-Gyro I/F OFF_OFILM	SRVCLK can't OFF_OFILM */
		break ;
	case STB2_OISOFF_OFILM:
		RegReadA_Ofilm( CLKON_OFILM, &UcClkon_OFILM ) ;		/* 0x020B	PWM Clock OFF_OFILM + D-Gyro I/F ON_OFILM  */
		UcClkon_OFILM |= 0x05 ;
		RegWriteA_Ofilm( CLKON_OFILM,	UcClkon_OFILM ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		SelectGySleep_OFILM( OFF_OFILM ) ;				/* Gyro Wake Up */
//		RegWriteA_Ofilm( DACMONFC, 0x81 ) ;		// 0x0032	DAC Monitor Active
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;		/* 0x0030	Monitor Active	*/
		DrvSw_OFILM( ON_OFILM ) ;						/* Driver Mode setting */
		RegWriteA_Ofilm( CVA_OFILM, 	0xC0 );			// 0x0020	Linear PWM mode enable
		RegWriteA_Ofilm( PWMA_OFILM	, 	0xC0 );			// 0x0010	PWM enable
		RegReadA_Ofilm( STBB0_OFILM	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0_OFILM |= 0x5F ;
		RegWriteA_Ofilm( STBB0_OFILM	, UcStbb0_OFILM );	// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb2 OISOff \n" ) ;
		break ;
		
	case STB2_AFON_OFILM:
TRACE( " STB MODE = Stb2 AFON \n" ) ;
#ifdef	AF_PWMMODE
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#endif
		RegReadA_Ofilm( STBB0_OFILM 	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0_OFILM &= 0x7F ;
		RegWriteA_Ofilm( STBB0_OFILM 	, UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		RegWriteA_Ofilm( STBB1_OFILM 	, 0x00 );		// 0x0264 	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		AfDrvSw_OFILM( OFF_OFILM ) ;					/* AF Drvier Block Ena=0 */
#ifdef	MONITOR_OFF_OFILM
#else
		RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;		// 0x0030	Monitor Standby
#endif
		RegReadA_Ofilm( CLKON_OFILM, &UcClkon_OFILM ) ;		/* 0x020B	OPAF Clock OFF_OFILM + AFPWM OFF_OFILM	SRVCLK can't OFF_OFILM	*/
		UcClkon_OFILM &= 0x07 ;
		RegWriteA_Ofilm( CLKON_OFILM, UcClkon_OFILM ) ;		/* 0x020B	OPAF Clock OFF_OFILM + AFPWM OFF_OFILM	SRVCLK can't OFF_OFILM	*/
		break ;
	case STB2_AFOFF_OFILM:
		RegReadA_Ofilm( CLKON_OFILM, &UcClkon_OFILM ) ;		/* 0x020B	OPAF Clock ON_OFILM + AFPWM ON_OFILM  */
		UcClkon_OFILM |= 0x18 ;
		RegWriteA_Ofilm( CLKON_OFILM,	UcClkon_OFILM ) ;		// 0x020B	[ - | - | CmOpafClkOn | CmAfpwmClkOn | CMGifClkOn  | CmScmClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
		AfDrvSw_OFILM( ON_OFILM ) ;						/* AF Driver Mode setting */
		RegWriteA_Ofilm( PWMAAF_OFILM 	, 0x00 );		// 0x0090		AF PWM Standby
		RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;		// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]
		RegReadA_Ofilm( STBB0_OFILM	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
		UcStbb0_OFILM |= 0x80 ;
		RegWriteA_Ofilm( STBB0_OFILM	, UcStbb0_OFILM );	// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
TRACE( " STB MODE = Stb2 AFOff \n" ) ;
		break ;
		/************** special mode ************/
	}
}
#endif

//********************************************************************************
// Function Name 	: SetZsp_OFILM
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Zoom Step parameter Function
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	SetZsp_OFILM( unsigned char	UcZoomStepDat_OFILM )
{
	unsigned long	UlGyrZmx_OFILM, UlGyrZmy_OFILM, UlGyrZrx_OFILM, UlGyrZry_OFILM ;

	
	/* Zoom Step */
	if(UcZoomStepDat_OFILM > (ZOOMTBL_OFILM - 1))
		UcZoomStepDat_OFILM = (ZOOMTBL_OFILM -1) ;										/* \8F\E3\8C\C0\82\F0ZOOMTBL_OFILM-1\82ɐݒ肷\82\E9 */

	if( UcZoomStepDat_OFILM == 0 )				/* initial setting	*/
	{
		UlGyrZmx_OFILM	= ClGyxZom_OFILM[ 0 ] ;		// Same Wide Coefficient
		UlGyrZmy_OFILM	= ClGyyZom_OFILM[ 0 ] ;		// Same Wide Coefficient
		/* Initial Rate value = 1 */
TRACE("Initial ZoomX = %08xh", (unsigned int)UlGyrZmx_OFILM ) ;
TRACE("        ZoomY = %08xh\n", (unsigned int)UlGyrZmy_OFILM ) ;
	}
	else
	{
		UlGyrZmx_OFILM	= ClGyxZom_OFILM[ UcZoomStepDat_OFILM ] ;
		UlGyrZmy_OFILM	= ClGyyZom_OFILM[ UcZoomStepDat_OFILM ] ;
		
		
TRACE("   Step ZoomX = %08xh", (unsigned int)UlGyrZmx_OFILM ) ;
TRACE("        ZoomY = %08xh", (unsigned int)UlGyrZmy_OFILM ) ;
TRACE("        Step = %d\n", UcZoomStepDat_OFILM ) ;
	}
	
	// Zoom Value Setting
	RamWrite32A_Ofilm( gxlens_OFILM, UlGyrZmx_OFILM ) ;		/* 0x1022 */
	RamWrite32A_Ofilm( gylens_OFILM, UlGyrZmy_OFILM ) ;		/* 0x1122 */

	RamRead32A_Ofilm( gxlens_OFILM, &UlGyrZrx_OFILM ) ;		/* 0x1022 */
	RamRead32A_Ofilm( gylens_OFILM, &UlGyrZry_OFILM ) ;		/* 0x1122 */

	// Zoom Value Setting Error Check
	if( UlGyrZmx_OFILM != UlGyrZrx_OFILM ) {
		RamWrite32A_Ofilm( gxlens_OFILM, UlGyrZmx_OFILM ) ;		/* 0x1022 */
	}

	if( UlGyrZmy_OFILM != UlGyrZry_OFILM ) {
		RamWrite32A_Ofilm( gylens_OFILM, UlGyrZmy_OFILM ) ;		/* 0x1122 */
	}

}

//********************************************************************************
// Function Name 	: StbOnn_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stabilizer For Servo On Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
 
void StbOnn_OFILM( void )
{
	unsigned char	UcRegValx_OFILM,UcRegValy_OFILM;					// Registor value 
	unsigned char	UcRegIni_OFILM ;
	unsigned char	UcRegIniCnt_OFILM = 0;
                
	RegReadA_Ofilm( WH_EQSWX_OFILM , &UcRegValx_OFILM ) ;			// 0x0170
	RegReadA_Ofilm( WH_EQSWY_OFILM , &UcRegValy_OFILM ) ;			// 0x0171
	
	if( (( UcRegValx_OFILM & 0x01 ) != 0x01 ) && (( UcRegValy_OFILM & 0x01 ) != 0x01 ))
	{
TRACE( " SMOOTH S \n" ) ;
		
		RegWriteA_Ofilm( WH_SMTSRVON_OFILM,	0x01 ) ;				// 0x017C		Smooth Servo ON_OFILM
		
		SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;
		SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;
		
		UcRegIni_OFILM = 0x11;
		while( (UcRegIni_OFILM & 0x77) != 0x66 )
		{
			RegReadA_Ofilm( RH_SMTSRVSTT_OFILM,	&UcRegIni_OFILM ) ;		// 0x01F8		Smooth Servo phase read
			
			if( CmdRdChk_OFILM() !=0 )	break;				// Dead Lock check (responce check)
			if((UcRegIni_OFILM & 0x77 ) == 0 )	UcRegIniCnt_OFILM++ ;
			if( UcRegIniCnt_OFILM > 10 ){
TRACE( " Slope Error \n" ) ;
				break ;			// Status Error
			}
			
		}
		RegWriteA_Ofilm( WH_SMTSRVON_OFILM,	0x00 ) ;				// 0x017C		Smooth Servo OFF_OFILM
		
	}
	else
	{
		SrvCon_OFILM( X_DIR_OFILM, ON_OFILM ) ;
		SrvCon_OFILM( Y_DIR_OFILM, ON_OFILM ) ;
TRACE( " Not Slope \n" ) ;
	}
}

//********************************************************************************
// Function Name 	: StbOnnN_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stabilizer For Servo On Function
// History			: First edition 						2013.10.09 Y.Shigeoka
//********************************************************************************
 
void StbOnnN_OFILM( unsigned char UcStbY_OFILM , unsigned char UcStbX_OFILM )
{
	unsigned char	UcRegIni_OFILM ;
	unsigned char	UcSttMsk_OFILM = 0 ;
	unsigned char	UcRegIniCnt_OFILM = 0;
	
TRACE( " SMOOTHN S \n" ) ;
	
	RegWriteA_Ofilm( WH_SMTSRVON_OFILM,	0x01 ) ;				// 0x017C		Smooth Servo ON_OFILM
	if( UcStbX_OFILM == ON_OFILM )	UcSttMsk_OFILM |= 0x07 ;
	if( UcStbY_OFILM == ON_OFILM )	UcSttMsk_OFILM |= 0x70 ;
	
	SrvCon_OFILM( X_DIR_OFILM, UcStbX_OFILM ) ;
	SrvCon_OFILM( Y_DIR_OFILM, UcStbY_OFILM ) ;
	
	UcRegIni_OFILM = 0x11;
	while( (UcRegIni_OFILM & UcSttMsk_OFILM) != ( 0x66 & UcSttMsk_OFILM ) )
	{
		RegReadA_Ofilm( RH_SMTSRVSTT_OFILM,	&UcRegIni_OFILM ) ;		// 0x01F8		Smooth Servo phase read
		
		if( CmdRdChk_OFILM() !=0 )	break;				// Dead Lock check (responce check)
		if((UcRegIni_OFILM & 0x77 ) == 0 )	UcRegIniCnt_OFILM++ ;
		if( UcRegIniCnt_OFILM > 10 ){
TRACE( " Slope Error \n" ) ;
			break ;			// Status Error
		}
		
	}
	RegWriteA_Ofilm( WH_SMTSRVON_OFILM,	0x00 ) ;				// 0x017C		Smooth Servo OFF_OFILM

}

//********************************************************************************
// Function Name 	: OptCen_OFILM
// Retun Value		: NON
// Argment Value	: UcOptMode 0:Set 1:Save&Set
//					: UsOptXval_OFILM Xaxis offset
//					: UsOptYval_OFILM Yaxis offset
// Explanation		: Send Optical Center
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
void	OptCen_OFILM( unsigned char UcOptmode_OFILM , unsigned short UsOptXval_OFILM , unsigned short UsOptYval_OFILM )
{
	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	switch ( UcOptmode_OFILM ) {
		case VAL_SET_OFILM :
			RamWriteA_Ofilm( SXOFFZ1_OFILM   , UsOptXval_OFILM ) ;		/* 0x1461	Check Hall X optical center */
			RamWriteA_Ofilm( SYOFFZ1_OFILM   , UsOptYval_OFILM ) ;		/* 0x14E1	Check Hall Y optical center */
TRACE("	SXOFFZ1_OFILM = %04xh , 	SYOFFZ1_OFILM = %04xh\n", UsOptXval_OFILM , UsOptYval_OFILM ) ;
TRACE(" Set Opt offset\n");
			break ;
		case VAL_FIX_OFILM :
			UsCntXof_OFILM = UsOptXval_OFILM ;
			UsCntYof_OFILM = UsOptYval_OFILM ;
			RamWriteA_Ofilm( SXOFFZ1_OFILM   , UsCntXof_OFILM ) ;		/* 0x1461	Check Hall X optical center */
			RamWriteA_Ofilm( SYOFFZ1_OFILM   , UsCntYof_OFILM ) ;		/* 0x14E1	Check Hall Y optical center */

TRACE("	SXOFFZ1_OFILM = %04xh , 	SYOFFZ1_OFILM = %04xh\n", UsCntXof_OFILM , UsCntYof_OFILM ) ;
TRACE(" Fix Opt offset\n");
			break ;
		case VAL_SPC_OFILM :
			RamReadA_Ofilm( SXOFFZ1_OFILM   , &UsOptXval_OFILM ) ;		/* 0x1461	Check Hall X optical center */
			RamReadA_Ofilm( SYOFFZ1_OFILM   , &UsOptYval_OFILM ) ;		/* 0x14E1	Check Hall Y optical center */
			UsCntXof_OFILM = UsOptXval_OFILM ;
			UsCntYof_OFILM = UsOptYval_OFILM ;


TRACE("	SXOFFZ1_OFILM = %04xh , 	SYOFFZ1_OFILM = %04xh\n", UsCntXof_OFILM , UsCntYof_OFILM ) ;
TRACE(" SPC_OFILM Opt offset\n");
			break ;
	}

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
}

#ifdef	MODULE_CALIBRATION_OFILM
//********************************************************************************
// Function Name 	: OscAdj_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: OSC Clock adjustment
// History			: First edition 						2013.01.15 Y.Shigeoka
//********************************************************************************
#define	RRATETABLE_OFILM	8
#define	CRATETABLE_OFILM	16
const signed char	ScRselRate_OFILM[ RRATETABLE_OFILM ]	= {
		-12,			/* -12% */
		 -9,			/*  -9% */
		 -6,			/*  -6% */
		 -3,			/*  -3% */
		  0,			/*   0% */
		  3,			/*   3% */
		  7,			/*   7% */
		 11				/*  11% */
	} ;
const signed char	ScCselRate_OFILM[ CRATETABLE_OFILM ]	= {
		-14,			/* -14% */
		-12,			/* -12% */
		-10,			/* -10% */
		 -8,			/*  -8% */
		 -6,			/*  -6% */
		 -4,			/*  -4% */
		 -2,			/*  -2% */
		  0,			/*   0% */
		  0,			/*   0% */
		  2,			/*   2% */
		  4,			/*   4% */
		  6,			/*   6% */
		  8,			/*   8% */
		 10,			/*  10% */
		 12,			/*  12% */
		 14				/*  14% */
	} ;
	

#define	TARGET_FREQ_OFILM		48000.0F/* 48MHz */
//#define	TARGET_FREQ_OFILM		24000.0F/* 24MHz */
#define	START_RSEL_OFILM		0x04	/* Typ */
#define	START_CSEL_OFILM		0x08	/* Typ bit4:OSCPMSEL */
#define	MEAS_MAX_OFILM		32		/* \8F\E3\8C\C032\89\F1 */
/* Measure Status (UcClkJdg_OFILM) */
#define	UNDR_MEAS_OFILM		0x00
#define	FIX_MEAS_OFILM		0x01
#define	JST_FIX_OFILM			0x03
#define	OVR_MEAS_OFILM		0x80
/* Measure Check Flag (UcMeasFlg_OFILM) */
#define	RSELFX_OFILM			0x08
#define	RSEL1ST_OFILM			0x01
#define	RSEL2ND_OFILM			0x02
#define	CSELFX_OFILM			0x80
#define	CSELPLS_OFILM			0x10
#define	CSELMNS_OFILM			0x20

unsigned short	OscAdj_OFILM( void )
{
	unsigned char	UcMeasFlg_OFILM ;									/* Measure check flag */
	UnWrdVal_OFILM		StClkVal_OFILM ;									/* Measure value */
	unsigned char	UcMeasCnt_OFILM ;									/* Measure counter */
	unsigned char	UcOscrsel_OFILM , UcOsccsel_OFILM ;						/* Reg set value */
	unsigned char	UcSrvDivBk_OFILM ;								/* back up value */
	unsigned char	UcClkJdg_OFILM ;									/* State flag */
	float			FcalA_OFILM,FcalB_OFILM ;								/* calcurate value */
	signed char		ScTblRate_Val_OFILM, ScTblRate_Now_OFILM, ScTblRate_Tgt_OFILM ;	/* rate */
	float			FlRatePbk_OFILM,FlRateMbk_OFILM ;							/* Rate bk  */
	unsigned char	UcOsccselP_OFILM , UcOsccselM_OFILM ;					/* Reg set value */
	unsigned short	UsResult_OFILM ;

//	unsigned char	UcOscsetBk ;								/* Reg set value */
	
	UcMeasFlg_OFILM = 0 ;						/* Clear Measure check flag */
	UcMeasCnt_OFILM = 0;						/* Clear Measure counter */
	UcClkJdg_OFILM = UNDR_MEAS_OFILM;				/* under Measure */
	UcOscrsel_OFILM = START_RSEL_OFILM ;
	UcOsccsel_OFILM = START_CSEL_OFILM ;
	/* check */
//	RegReadA_Ofilm( OSCSET_OFILM, &UcOscsetBk ) ;	// 0x0264	
//	UcOscrsel_OFILM = ( UcOscsetBk & 0xE0 ) >> 5 ;
//	UcOsccsel_OFILM = ( UcOscsetBk & 0x1E ) >> 1 ;
	/**/
	
	RegReadA_Ofilm( SRVDIV_OFILM, &UcSrvDivBk_OFILM ) ;	/* 0x0211 */
	RegWriteA_Ofilm( SRVDIV_OFILM,	0x00 ) ;		// 0x0211	 SRV Clock = Xtalck
	RegWriteA_Ofilm( OSCSET_OFILM, ( UcOscrsel_OFILM << 5 ) | (UcOsccsel_OFILM << 1 ) ) ;	// 0x0257	 
TRACE("	RSEL   = %02xh",  UcOscrsel_OFILM ) ;
TRACE("	CSEL   = %02xh (Initial)\n",  UcOsccsel_OFILM ) ;
//TRACE("	OSCSET_OFILM   = %02xh\n",  UcOscsetBk ) ;
	
	while( UcClkJdg_OFILM == UNDR_MEAS_OFILM )
	{
		UcMeasCnt_OFILM++ ;						/* Measure count up */
		UcOscAdjFlg_OFILM = MEASSTR_OFILM ;				// Start trigger ON_OFILM

		while( UcOscAdjFlg_OFILM != MEASFIX_OFILM )
		{
			;
		}
		
		UcOscAdjFlg_OFILM = 0x00 ;				// Clear Flag
		RegReadA_Ofilm( OSCCK_CNTR0_OFILM, &StClkVal_OFILM.StWrdVal_OFILM.UcLowVal_OFILM ) ;		/* 0x025E */
		RegReadA_Ofilm( OSCCK_CNTR1_OFILM, &StClkVal_OFILM.StWrdVal_OFILM.UcHigVal_OFILM ) ;		/* 0x025F */
		
TRACE("		COUNT = %02xh , OSC CNT = %04xh\n", UcMeasCnt_OFILM , StClkVal_OFILM.UsWrdVal_OFILM ) ;
TRACE("		CNT1 = %02xh ,	CNT0 = %02xh \n", StClkVal_OFILM.StWrdVal_OFILM.UcHigVal_OFILM , StClkVal_OFILM.StWrdVal_OFILM.UcLowVal_OFILM ) ;
		FcalA_OFILM = (float)StClkVal_OFILM.UsWrdVal_OFILM ;
		FcalB_OFILM = TARGET_FREQ_OFILM / FcalA_OFILM ;
		FcalB_OFILM =  FcalB_OFILM - 1.0F ;
		FcalB_OFILM *= 100.0F ;
		
		if( FcalB_OFILM == 0.0F )
		{
			UcClkJdg_OFILM = JST_FIX_OFILM ;					/* Just 36MHz */
			UcMeasFlg_OFILM |= ( CSELFX_OFILM | RSELFX_OFILM ) ;		/* Fix Flag */
TRACE("		************ JUST OK \n") ;
			break ;
		}

		/* Rsel check */
		if( !(UcMeasFlg_OFILM & RSELFX_OFILM) )
		{
			if(UcMeasFlg_OFILM & RSEL1ST_OFILM)
			{
				UcMeasFlg_OFILM |= ( RSELFX_OFILM | RSEL2ND_OFILM ) ;
			}
			else
			{
				UcMeasFlg_OFILM |= RSEL1ST_OFILM ;
			}
TRACE("	*** Rsel Check *** ") ;
			ScTblRate_Now_OFILM = ScRselRate_OFILM[ UcOscrsel_OFILM ] ;					/* \8D\A1\82\CCRate */
			ScTblRate_Tgt_OFILM = ScTblRate_Now_OFILM + (short)FcalB_OFILM ;
TRACE("	N = %02xh, T = %02xh\n", ScTblRate_Now_OFILM , ScTblRate_Tgt_OFILM) ;
			if( ScTblRate_Now_OFILM > ScTblRate_Tgt_OFILM )
			{
TRACE("	N>T ***** \n") ;
				while(1)
				{
					if( UcOscrsel_OFILM == 0 )
					{
						break;
					}
					UcOscrsel_OFILM -= 1 ;
					ScTblRate_Val_OFILM = ScRselRate_OFILM[ UcOscrsel_OFILM ] ;	
TRACE("				RSEL = %02xh,  Tgt = %02xh , Val = %02xh \n", UcOscrsel_OFILM,ScTblRate_Tgt_OFILM,ScTblRate_Val_OFILM ) ;
					if( ScTblRate_Tgt_OFILM >= ScTblRate_Val_OFILM )
					{
						break;
					}
				}
			}
			else if( ScTblRate_Now_OFILM < ScTblRate_Tgt_OFILM )
			{
TRACE("	N<T ***** \n") ;
				while(1)
				{
					if(UcOscrsel_OFILM == (RRATETABLE_OFILM - 1))
					{
						break;
					}
					UcOscrsel_OFILM += 1 ;
					ScTblRate_Val_OFILM = ScRselRate_OFILM[ UcOscrsel_OFILM ] ;	
TRACE("				RSEL = %02xh,  Tgt = %02xh , Val = %02xh \n", UcOscrsel_OFILM,ScTblRate_Tgt_OFILM,ScTblRate_Val_OFILM ) ;
					if( ScTblRate_Tgt_OFILM <= ScTblRate_Val_OFILM )
					{
						break;
					}
				}
			}
			else
			{
TRACE("	N=T ***** \n") ;
				;
			}
		}
		else
		{		
		/* Csel check */
TRACE("	*** Csel Check *** ") ;
			if( FcalB_OFILM > 0 )			/* Plus */
			{
TRACE("		Plus \n") ;
				UcMeasFlg_OFILM |= CSELPLS_OFILM ;
				FlRatePbk_OFILM = FcalB_OFILM ;
				UcOsccselP_OFILM = UcOsccsel_OFILM ;
				if( UcMeasFlg_OFILM & CSELMNS_OFILM)
				{
					UcMeasFlg_OFILM |= CSELFX_OFILM ;
					UcClkJdg_OFILM = FIX_MEAS_OFILM ;			/* OK */
TRACE("		************ OK \n") ;
				}
				else if(UcOsccsel_OFILM == (CRATETABLE_OFILM - 1))
				{
					if(UcOscrsel_OFILM < ( RRATETABLE_OFILM - 1 ))
					{
						UcOscrsel_OFILM += 1 ;
						UcOsccsel_OFILM = START_CSEL_OFILM ;
						UcMeasFlg_OFILM = 0 ;			/* Clear */
TRACE("		************ Re-Start \n") ;
					}
					else
					{
						UcClkJdg_OFILM = OVR_MEAS_OFILM ;			/* Over */
TRACE("		************ OVER \n") ;
					}
				}
				else
				{
					UcOsccsel_OFILM += 1 ;
				}
			}
			else					/* Minus */
			{
TRACE("		Minus \n") ;
				UcMeasFlg_OFILM |= CSELMNS_OFILM ;
				FlRateMbk_OFILM = (-1)*FcalB_OFILM ;
				UcOsccselM_OFILM = UcOsccsel_OFILM ;
				if( UcMeasFlg_OFILM & CSELPLS_OFILM)
				{
					UcMeasFlg_OFILM |= CSELFX_OFILM ;
					UcClkJdg_OFILM = FIX_MEAS_OFILM ;			/* OK */
TRACE("		************ OK \n") ;
				}
				else if(UcOsccsel_OFILM == 0x00)
				{
					if(UcOscrsel_OFILM > 0)
					{
						UcOscrsel_OFILM -= 1 ;
						UcOsccsel_OFILM = START_CSEL_OFILM ;
						UcMeasFlg_OFILM = 0 ;			/* Clear */
TRACE("		************ Re-Start \n") ;
					}
					else
					{
					UcClkJdg_OFILM = OVR_MEAS_OFILM ;			/* Over */
TRACE("		************ OVER \n") ;
					}
				}
				else
				{
					UcOsccsel_OFILM -= 1 ;
				}
			}
			if(UcMeasCnt_OFILM >= MEAS_MAX_OFILM)
			{
				UcClkJdg_OFILM = OVR_MEAS_OFILM ;			/* Over */
TRACE("		************ TIME OVER \n") ;
			}
		}	
TRACE("		RSEL = %02xh, CSEL = %02xh\n", UcOscrsel_OFILM, UcOsccsel_OFILM ) ;
		RegWriteA_Ofilm( OSCSET_OFILM, ( UcOscrsel_OFILM << 5 ) | (UcOsccsel_OFILM << 1 ) ) ;	// 0x0257	 
	}
	
	UsResult_OFILM = EXE_END_OFILM ;
	
	if(UcClkJdg_OFILM == FIX_MEAS_OFILM)
	{
		if( FlRatePbk_OFILM < FlRateMbk_OFILM )
		{
			UcOsccsel_OFILM = UcOsccselP_OFILM ; 
		}
		else
		{
			UcOsccsel_OFILM = UcOsccselM_OFILM ; 
		}
	
TRACE("	FIX *****>	RSEL = %02xh, CSEL = %02xh\n", UcOscrsel_OFILM, UcOsccsel_OFILM ) ;
		RegWriteA_Ofilm( OSCSET_OFILM, ( UcOscrsel_OFILM << 5 ) | (UcOsccsel_OFILM << 1 ) ) ;	// 0x0264	 

		/* check */
//		RegReadA_Ofilm( OSCSET_OFILM, &UcOscsetBk ) ;	// 0x0257	
//TRACE("	OSCSET_OFILM   = %02xh\n",  UcOscsetBk ) ;
		
	}
	StAdjPar_OFILM.UcOscVal_OFILM = ( ( UcOscrsel_OFILM << 5 ) | (UcOsccsel_OFILM << 1 ) );
	
	if(UcClkJdg_OFILM == OVR_MEAS_OFILM)
	{
		UsResult_OFILM = EXE_OCADJ_OFILM ;
		StAdjPar_OFILM.UcOscVal_OFILM = 0x00 ;
	}
	RegWriteA_Ofilm( SRVDIV_OFILM,	UcSrvDivBk_OFILM ) ;		// 0x0211	 SRV Clock set
	return( UsResult_OFILM );
}
#endif


#ifdef HALLADJ_HW_OFILM
//==============================================================================
//  Function    :   SetSineWave_OFILM()
//  inputs      :   UcJikuSel_OFILM   0: X-Axis
//                              1: Y-Axis
//                  UcMeasMode_OFILM  0: Loop Gain frequency setting
//                              1: Bias/Offset frequency setting
//  outputs     :   void
//  explanation :   Initializes sine wave settings:
//                      Sine Table, Amplitue, Offset, Frequency
//  revisions   :   First Edition                          2013.01.15 Y.Shigeoka
//==============================================================================
void SetSineWave_OFILM( unsigned char UcJikuSel_OFILM , unsigned char UcMeasMode_OFILM )
{
 #ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
    unsigned short  UsFRQ_OFILM[]   = { 0x30EE/*139.9Hz*/ , 0x037E/*10Hz*/ } ;          // { Loop Gain setting , Bias/Offset setting}
 #else
    unsigned short  UsFRQ_OFILM[]   = { 0x1877/*139.9Hz*/ , 0x01BF/*10Hz*/ } ;          // { Loop Gain setting , Bias/Offset setting}
 #endif
	unsigned long   UlAMP_OFILM[2][2]   = {{ 0x3CA3D70A , 0x3CA3D70A } ,		// Loop Gain   { X amp , Y amp }
									 { 0x3F800000 , 0x3F800000 } };		// Bias/offset { X amp , Y amp }
	unsigned char	UcEqSwX_OFILM , UcEqSwY_OFILM ;

    UcMeasMode_OFILM &= 0x01;
    UcJikuSel_OFILM  &= 0x01;

	/* Phase parameter 0deg */
	RegWriteA_Ofilm( WC_SINPHSX_OFILM, 0x00 ) ;					/* 0x0183	*/
	RegWriteA_Ofilm( WC_SINPHSY_OFILM, 0x00 ) ;					/* 0x0184	*/
	
	/* wait 0 cross */
	RegWriteA_Ofilm( WC_MESSINMODE_OFILM,     0x00 ) ;			/* 0x0191	Sine 0 cross  */
	RegWriteA_Ofilm( WC_MESWAIT_OFILM,     0x00 ) ;				/* 0x0199	0 cross wait */
	
    /* Manually Set Amplitude */
	RamWrite32A_Ofilm( sxsin_OFILM, UlAMP_OFILM[UcMeasMode_OFILM][X_DIR_OFILM] ) ;				// 0x10D5
	RamWrite32A_Ofilm( sysin_OFILM, UlAMP_OFILM[UcMeasMode_OFILM][Y_DIR_OFILM] ) ;				// 0x11D5

	/* Freq */
	RegWriteA_Ofilm( WC_SINFRQ0_OFILM,	(unsigned char)UsFRQ_OFILM[UcMeasMode_OFILM] ) ;				// 0x0181		Freq L
	RegWriteA_Ofilm( WC_SINFRQ1_OFILM,	(unsigned char)(UsFRQ_OFILM[UcMeasMode_OFILM] >> 8) ) ;			// 0x0182		Freq H

    /* Clear Optional Sine wave input address */
	RegReadA_Ofilm( WH_EQSWX_OFILM, &UcEqSwX_OFILM ) ;				/* 0x0170	*/
	RegReadA_Ofilm( WH_EQSWY_OFILM, &UcEqSwY_OFILM ) ;				/* 0x0171	*/
    if( !UcMeasMode_OFILM && !UcJikuSel_OFILM )       // Loop Gain mode  X-axis
    {
		UcEqSwX_OFILM |= 0x10 ;				/* SW[4] */
		UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
    }
    else if( !UcMeasMode_OFILM && UcJikuSel_OFILM )   // Loop Gain mode Y-Axis
    {
		UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
		UcEqSwY_OFILM |= 0x10 ;				/* SW[4] */
    }
    else if( UcMeasMode_OFILM && !UcJikuSel_OFILM )   // Bias/Offset mode X-Axis
    {
    	UcEqSwX_OFILM = 0x22 ;				/* SW[5] */
    	UcEqSwY_OFILM = 0x03 ;
    }
    else                    // Bias/Offset mode Y-Axis
    {
		UcEqSwX_OFILM = 0x03 ;
		UcEqSwY_OFILM = 0x22 ;				/* SW[5] */

    }
	RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	*/
	RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	*/
}

//==============================================================================
//  Function    :   StartSineWave_OFILM()
//  inputs      :   none
//  outputs     :   void
//  explanation :   Starts sine wave
//  revisions   :   First Edition                          2011.04.13 d.yamagata
//==============================================================================
void StartSineWave_OFILM( void )
{
    /* Start Sine Wave */
	RegWriteA_Ofilm( WC_SINON_OFILM,     0x01 ) ;				/* 0x0180	Sine wave  */

}

//==============================================================================
//  Function    :   StopSineWave_OFILM()
//  inputs      :   void
//  outputs     :   void
//  explanation :   Stops sine wave
//  revisions   :   First Edition                          2013.01.15 Y.Shigeoka
//==============================================================================
void StopSineWave_OFILM( void )
{
	unsigned char		UcEqSwX_OFILM , UcEqSwY_OFILM ;
	
	RegWriteA_Ofilm( WC_SINON_OFILM,     0x00 ) ;				/* 0x0180	Sine wave Stop */
	RegReadA_Ofilm( WH_EQSWX_OFILM, &UcEqSwX_OFILM ) ;				/* 0x0170	*/
	RegReadA_Ofilm( WH_EQSWY_OFILM, &UcEqSwY_OFILM ) ;				/* 0x0171	*/
	UcEqSwX_OFILM &= ~EQSINSW_OFILM ;
	UcEqSwY_OFILM &= ~EQSINSW_OFILM ;
	RegWriteA_Ofilm( WH_EQSWX_OFILM, UcEqSwX_OFILM ) ;				/* 0x0170	Switch control */
	RegWriteA_Ofilm( WH_EQSWY_OFILM, UcEqSwY_OFILM ) ;				/* 0x0171	Switch control */

}

//==============================================================================
//  Function    :   SetMeaseFil_LoopGain()
//  inputs      :   UcJikuSel_OFILM   0: X-Axis
//                              1: Y-Axis
//                  UcMeasMode_OFILM  0: Loop Gain frequency setting
//                              1: Bias/Offset frequency setting
//                  UcFilSel_OFILM
//  outputs     :   void
//  explanation :
//  revisions   :   First Edition                          2013.01.15 Y.Shigeoka
//==============================================================================
void SetMeasFil_OFILM( unsigned char UcFilSel_OFILM )
{
	
	MesFil_OFILM( UcFilSel_OFILM ) ;					/* Set Measure filter */

}

//==============================================================================
//  Function    :   ClrMeasFil_OFILM()
//  inputs      :   void
//  outputs     :   void
//  explanation :
//  revisions   :   First Edition                          2013.01.15 Y.Shigeoka
//==============================================================================
void ClrMeasFil_OFILM( void )
{
    /* Measure Filters clear */
	ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );		// MEAS-FIL Delay RAM Clear
 
}

 #ifdef	MODULE_CALIBRATION_OFILM
//==============================================================================
//  Function    :   LoopGainAdj_OFILM()
//  inputs      :   UcJikuSel_OFILM   0: X-Axis, 1: Y-Axis
//  outputs     :   void
//  explanation :
//  revisions   :   First Edition                          2011.04.13 d.yamagata
//==============================================================================
unsigned char	 LoopGainAdj_OFILM( unsigned char UcJikuSel_OFILM)
{

	unsigned short	UsRltVal_OFILM ;
	unsigned char	UcAdjSts_OFILM	= FAILURE_OFILM ;
	
    UcJikuSel_OFILM &= 0x01;

	StbOnn_OFILM() ;											// Slope Mode
	
	// Wait 200ms
	WitTim_Ofilm( 200 ) ;
	
	/* set start gain */
	LopPar_OFILM( UcJikuSel_OFILM ) ;
	
	/* set sine wave */
    SetSineWave_OFILM( UcJikuSel_OFILM , __MEASURE_LOOPGAIN_OFILM );

	/* Measure count */
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x00 ) ;				// 0x0193
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x40 ) ;				// 0x0192	64 Times Measure
	RamWrite32A_Ofilm( msmean_OFILM	, 0x3C800000 );				// 0x1230	1/CmMesLoop[15:0]
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x01 ) ;					// 0x0198	ABS
	
    /* Set Adjustment Limits */
    RamWrite32A_Ofilm( LGMax_OFILM  , 0x3F800000 );		// 0x1092	Loop gain adjustment max limit
    RamWrite32A_Ofilm( LGMin_OFILM  , 0x3E000100 );		// 0x1091	Loop gain adjustment min limit
    RegWriteA_Ofilm( WC_AMJLOOP1_OFILM, 0x00 );			// 0x01A3	Time-Out time
    RegWriteA_Ofilm( WC_AMJLOOP0_OFILM, 0x41 );			// 0x01A2	Time-Out time
    RegWriteA_Ofilm( WC_AMJIDL1_OFILM, 0x00 );			// 0x01A5	wait
    RegWriteA_Ofilm( WC_AMJIDL0_OFILM, 0x00 );			// 0x01A4	wait

    /* set Measure Filter */
    SetMeasFil_OFILM( LOOPGAIN_OFILM );

    /* Clear Measure Filters */
    ClrMeasFil_OFILM();

    /* Start Sine Wave */
    StartSineWave_OFILM();

    /* Enable Loop Gain Adjustment */
    /* Check Busy Flag */
TRACE("Loop Gain Auto Adjust Start.....\n\n");
	BsyWit_OFILM( WC_AMJMODE_OFILM, (0x0E | ( UcJikuSel_OFILM << 4 )) ) ;				// 0x01A0	Loop Gain adjustment
TRACE("Loop Gain Auto Adjust Complete!\n\n");

	RegReadA_Ofilm( RC_AMJERROR_OFILM, &UcAdjBsy_OFILM ) ;							// 0x01AD

	/* Ram Access */
	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	if( UcAdjBsy_OFILM )
	{
		if( UcJikuSel_OFILM == X_DIR_OFILM )
		{
			RamReadA_Ofilm( sxg_OFILM, &UsRltVal_OFILM ) ;						// 0x10D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM	= UsRltVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgSts_OFILM	= 0x0000 ;
TRACE("		sxg_OFILM = %04xh\n",UsRltVal_OFILM );
		} else {
			RamReadA_Ofilm( syg_OFILM, &UsRltVal_OFILM ) ;						// 0x11D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM	= UsRltVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLygSts_OFILM	= 0x0000 ;
TRACE("		syg_OFILM = %04xh\n",UsRltVal_OFILM );
		}

TRACE(" <<<********* ERROR !\n\n");
	}
	else
	{
		if( UcJikuSel_OFILM == X_DIR_OFILM )
		{
			RamReadA_Ofilm( sxg_OFILM, &UsRltVal_OFILM ) ;						// 0x10D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgVal_OFILM	= UsRltVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLxgSts_OFILM	= 0xFFFF ;
TRACE("		sxg_OFILM = %04xh\n",UsRltVal_OFILM );
		} else {
			RamReadA_Ofilm( syg_OFILM, &UsRltVal_OFILM ) ;						// 0x11D3
			StAdjPar_OFILM.StLopGan_OFILM.UsLygVal_OFILM	= UsRltVal_OFILM ;
			StAdjPar_OFILM.StLopGan_OFILM.UsLygSts_OFILM	= 0xFFFF ;
TRACE("		syg_OFILM = %04xh\n",UsRltVal_OFILM );
		}
		UcAdjSts_OFILM	= SUCCESS_OFILM ;												// Status OK
TRACE(" <<<********* OK !\n\n");
	}

	/* Ram Access */
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
    /* Stop Sine Wave */
    StopSineWave_OFILM();

	return( UcAdjSts_OFILM ) ;
}
 #endif

//==============================================================================
//  Function    :   BiasOffsetAdj_OFILM()
//  inputs      :   UcJikuSel_OFILM   0: X-Axis, 1: Y-Axis	UcMeasCnt_OFILM  :Measure Count
//  outputs     :   Result status
//  explanation :
//  revisions   :   First Edition                          2013.01.16 Y.Shigeoka
//==============================================================================
unsigned char  BiasOffsetAdj_OFILM( unsigned char UcJikuSel_OFILM , unsigned char UcMeasCnt_OFILM )
{
	unsigned char 	UcHadjRst_OFILM ;
									/*	   STEP         OFSTH        OFSTL        AMPH         AMPL                 (80%)*/
	unsigned long  UlTgtVal_OFILM[2][5]    =  {{ 0x3F800000 , 0x3D200140 , 0xBD200140 , 0x3F547AE1 , 0x3F451EB8 },	/* ROUGH */
										 { 0x3F000000 , 0x3D200140 , 0xBD200140 , 0x3F50A3D7 , 0x3F48F5C3 }} ;	/* FINE */

	if(UcMeasCnt_OFILM > 1)		UcMeasCnt_OFILM = 1 ;
	
    UcJikuSel_OFILM &= 0x01;

    /* Set Sine Wave */
    SetSineWave_OFILM( UcJikuSel_OFILM , __MEASURE_BIASOFFSET_OFILM );

	/* Measure count */
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x00 ) ;				// 0x0193
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x04 ) ;				// 0x0192	4 Times Measure
	RamWrite32A_Ofilm( msmean_OFILM	, 0x3E000000 );				// 0x10AE	1/CmMesLoop[15:0]/2
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x00 ) ;					// 0x0198	ABS

    /* Set Adjustment Limits */
    RamWrite32A_Ofilm( HOStp_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][0] );		// 0x1070	Hall Offset Stp
    RamWrite32A_Ofilm( HOMax_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][1] );		// 0x1072	Hall Offset max limit
    RamWrite32A_Ofilm( HOMin_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][2] );		// 0x1071	Hall Offset min limit
    RamWrite32A_Ofilm( HBStp_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][0] );		// 0x1080	Hall Bias Stp
    RamWrite32A_Ofilm( HBMax_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][3] );		// 0x1082	Hall Bias max limit
    RamWrite32A_Ofilm( HBMin_OFILM  , UlTgtVal_OFILM[UcMeasCnt_OFILM][4] );		// 0x1081	Hall Bias min limit

	RegWriteA_Ofilm( WC_AMJLOOP1_OFILM, 0x00 );			// 0x01A3	Time-Out time
    RegWriteA_Ofilm( WC_AMJLOOP0_OFILM, 0x40 );			// 0x01A2	Time-Out time
    RegWriteA_Ofilm( WC_AMJIDL1_OFILM, 0x00 );			// 0x01A5	wait
    RegWriteA_Ofilm( WC_AMJIDL0_OFILM, 0x00 );			// 0x01A4	wait
	
    /* Set Measure Filter */
    SetMeasFil_OFILM( HALL_ADJ );

    /* Clear Measure Filters */
    ClrMeasFil_OFILM();

    /* Start Sine Wave */
    StartSineWave_OFILM();

    /* Check Busy Flag */
TRACE("Hall Bias/Offset Auto Adjust Start.....\n\n");
	BsyWit_OFILM( WC_AMJMODE_OFILM, (0x0C | ( UcJikuSel_OFILM << 4 )) ) ;				// 0x01A0	Hall bais/offset ppt adjustment
TRACE("Hall Bias/Offset Auto Adjust END!\n\n");
	
	RegReadA_Ofilm( RC_AMJERROR_OFILM, &UcAdjBsy_OFILM ) ;							// 0x01AD
	
	if( UcAdjBsy_OFILM )
	{
		if( UcJikuSel_OFILM == X_DIR_OFILM )
		{
			UcHadjRst_OFILM = EXE_HXADJ_OFILM ;
		}
		else
		{
			UcHadjRst_OFILM = EXE_HYADJ_OFILM ;
		}

TRACE(" <<<********* ERROR !\n\n");
	}
	else
	{
		UcHadjRst_OFILM = EXE_END_OFILM ;
	}

    /* Stop Sine Wave */
    StopSineWave_OFILM();

    /* Set Servo Filter */
	
	/* Ram Access */
	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	if( UcJikuSel_OFILM == X_DIR_OFILM )
	{
		RamReadA_Ofilm( MSPP1AV_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMxa_OFILM  ) ;	 	// 0x1042 Max width value
		RamReadA_Ofilm( MSCT1AV_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM  ) ;	 	// 0x1052 offset value
TRACE("    Xmxa = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxMxa_OFILM ) ;
TRACE("    Xcen = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM ) ;
	}
	else
	{
//		RamReadA_Ofilm( MSPP2AV, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMxa_OFILM  ) ;	 	// 0x1142 Max width value
//		RamReadA_Ofilm( MSCT2AV, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM  ) ;	 	// 0x1152 offset value
		RamReadA_Ofilm( MSPP1AV_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMxa_OFILM  ) ;	 	// 0x1042 Max width value
		RamReadA_Ofilm( MSCT1AV_OFILM, &StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM  ) ;	 	// 0x1052 offset value
TRACE("    Ymxa = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyMxa_OFILM ) ;
TRACE("    Ycen = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM ) ;
	}

	StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM = (unsigned short)((signed short)StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM << 1 ) ;
	StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM = (unsigned short)((signed short)StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM << 1 ) ;
TRACE("    XcenX2 = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM ) ;
TRACE("    YcenX2 = %04xh \n", StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM ) ;
	/* Ram Access */
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
	return( UcHadjRst_OFILM ) ;
}

#endif


//********************************************************************************
// Function Name 	: GyrGan_OFILM
// Retun Value		: NON
// Argment Value	: UcGygmode_OFILM 0:Set 1:Save&Set
//					: UlGygXval_OFILM Xaxis Gain
//					: UlGygYval_OFILM Yaxis Gain
// Explanation		: Send Gyro Gain
// History			: First edition 						2011.02.09 Y.Shigeoka
//********************************************************************************
void	GyrGan_OFILM( unsigned char UcGygmode_OFILM , unsigned long UlGygXval_OFILM , unsigned long UlGygYval_OFILM )
{
	switch ( UcGygmode_OFILM ) {
		case VAL_SET_OFILM :
			RamWrite32A_Ofilm( gxzoom_OFILM, UlGygXval_OFILM ) ;		// 0x1020
			RamWrite32A_Ofilm( gyzoom_OFILM, UlGygYval_OFILM ) ;		// 0x1120
TRACE(" Set Gyro Gain\n");
			break ;
		case VAL_FIX_OFILM :
			RamWrite32A_Ofilm( gxzoom_OFILM, UlGygXval_OFILM ) ;		// 0x1020
			RamWrite32A_Ofilm( gyzoom_OFILM, UlGygYval_OFILM ) ;		// 0x1120

TRACE(" Fix Gyro Gain\n");
			break ;
		case VAL_SPC_OFILM :
			RamRead32A_Ofilm( gxzoom_OFILM, &UlGygXval_OFILM ) ;		// 0x1020
			RamRead32A_Ofilm( gyzoom_OFILM, &UlGygYval_OFILM ) ;		// 0x1120
		
TRACE(" Spc Gyro Gain\n");
			break ;
	}

TRACE(" X Gyro gain = %08xh\n", (unsigned int)UlGygXval_OFILM ) ;
TRACE(" Y Gyro gain = %08xh\n", (unsigned int)UlGygYval_OFILM ) ;
}

//********************************************************************************
// Function Name 	: SetPanTiltMode_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	SetPanTiltMode_OFILM( unsigned char UcPnTmod_OFILM )
{
	switch ( UcPnTmod_OFILM ) {
		case OFF_OFILM :
			RegWriteA_Ofilm( WG_PANON_OFILM, 0x00 ) ;			// 0x0109	X,Y Pan/Tilt Function OFF_OFILM
TRACE(" PanTilt OFF_OFILM\n");
			break ;
		case ON_OFILM :
			RegWriteA_Ofilm( WG_PANON_OFILM, 0x01 ) ;			// 0x0109	X,Y Pan/Tilt Function ON_OFILM
//			RegWriteA_Ofilm( WG_PANON_OFILM, 0x10 ) ;			// 0x0109	X,Y New Pan/Tilt Function ON_OFILM
TRACE(" PanTilt ON_OFILM\n");
			break ;
	}

}


#ifdef GAIN_CONT_OFILM
//********************************************************************************
// Function Name 	: TriSts_OFILM
// Retun Value		: Tripod Status
//					: bit0( 1:Y Tripod ON_OFILM / 0:OFF_OFILM)
//					: bit4( 1:X Tripod ON_OFILM / 0:OFF_OFILM)
//					: bit7( 1:Tripod ENABLE  / 0:DISABLE)
// Argment Value	: NON
// Explanation		: Read Status of Tripod mode Function
// History			: First edition 						2013.02.18 Y.Shigeoka
//********************************************************************************
unsigned char	TriSts_OFILM( void )
{
	unsigned char UcRsltSts_OFILM = 0;
	unsigned char UcVal_OFILM ;

	RegReadA_Ofilm( WG_ADJGANGXATO_OFILM, &UcVal_OFILM ) ;	// 0x0129
	if( UcVal_OFILM & 0x03 ){						// Gain control enable?
		RegReadA_Ofilm( RG_LEVJUGE_OFILM, &UcVal_OFILM ) ;	// 0x01F4
		UcRsltSts_OFILM = UcVal_OFILM & 0x11 ;		// bit0, bit4 set
		UcRsltSts_OFILM |= 0x80 ;				// bit7 ON_OFILM
	}
	return( UcRsltSts_OFILM ) ;
}
#endif

//********************************************************************************
// Function Name 	: DrvPwmSw_OFILM
// Retun Value		: Mode Status
//					: bit4( 1:PWM / 0:LinearPwm)
// Argment Value	: NON
// Explanation		: Select Driver mode Function
// History			: First edition 						2013.02.18 Y.Shigeoka
//********************************************************************************
unsigned char	DrvPwmSw_OFILM( unsigned char UcSelPwmMod_OFILM )
{

	switch ( UcSelPwmMod_OFILM ) {
		case Mlnp_OFILM :
			RegWriteA_Ofilm( DRVFC_OFILM	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
			UcPwmMod_OFILM = PWMMOD_CVL_OFILM ;
			break ;
		
		case Mpwm_OFILM :
			RegWriteA_Ofilm( DRVFC_OFILM	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
			UcPwmMod_OFILM = PWMMOD_PWM_OFILM ;
 			break ;
	}
	
	return( UcSelPwmMod_OFILM << 4 ) ;
}

 #ifdef	NEUTRAL_CENTER_OFILM
//********************************************************************************
// Function Name 	: TneHvc_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset
// History			: First edition 						2013.03.13	T.Tokoro
//********************************************************************************
unsigned char	TneHvc_OFILM( void )
{
	unsigned char	UcRsltSts_OFILM;
	unsigned short	UsMesRlt1_OFILM ;
	unsigned short	UsMesRlt2_OFILM ;
	
	SrvCon_OFILM( X_DIR_OFILM, OFF_OFILM ) ;				// X Servo OFF_OFILM
	SrvCon_OFILM( Y_DIR_OFILM, OFF_OFILM ) ;				// Y Servo OFF_OFILM
	
	WitTim_Ofilm( 500 ) ;
	
	//\95\BD\8Bϒl\91\AA\92\E8
	
	MesFil_OFILM( THROUGH_OFILM ) ;					// Set Measure Filter
	
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM	, 0x00 );			// 0x0193	CmMesLoop[15:8]
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM	, 0x40);			// 0x0192	CmMesLoop[7:0]

	RamWrite32A_Ofilm( msmean_OFILM	, 0x3C800000 );			// 0x1230	1/CmMesLoop[15:0]
	
	RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )AD0Z_OFILM ) ;							/* 0x0194	*/
	RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( AD0Z_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
	RegWriteA_Ofilm( WC_MES2ADD0_OFILM,  ( unsigned char )AD1Z_OFILM ) ;							/* 0x0196	*/
	RegWriteA_Ofilm( WC_MES2ADD1_OFILM,  ( unsigned char )(( AD1Z_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0197	*/
	
	RamWrite32A_Ofilm( MSABS1AV_OFILM, 	0x00000000 ) ;		// 0x1041
	RamWrite32A_Ofilm( MSABS2AV_OFILM, 	0x00000000 ) ;		// 0x1141
	
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x00 ) ;				// 0x0198	none ABS
	
	BsyWit_OFILM( WC_MESMODE_OFILM, 0x01 ) ;				// 0x0190		Normal Measure
	
	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	
	RamReadA_Ofilm( MSABS1AV_OFILM, &UsMesRlt1_OFILM ) ;			// 0x1041	Measure Result
	RamReadA_Ofilm( MSABS2AV_OFILM, &UsMesRlt2_OFILM ) ;			// 0x1141	Measure Result

	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode
	
	StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCna_OFILM = UsMesRlt1_OFILM;			//Measure Result Store
	StAdjPar_OFILM.StHalAdj_OFILM.UsHlxCen_OFILM = UsMesRlt1_OFILM;			//Measure Result Store
	
	StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCna_OFILM = UsMesRlt2_OFILM;			//Measure Result Store
	StAdjPar_OFILM.StHalAdj_OFILM.UsHlyCen_OFILM = UsMesRlt2_OFILM;			//Measure Result Store
	
	UcRsltSts_OFILM = EXE_END_OFILM ;				// Clear Status
	
	return( UcRsltSts_OFILM );
}
 #endif	//NEUTRAL_CENTER_OFILM

//********************************************************************************
// Function Name 	: SetGcf_OFILM
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set DI filter coefficient Function
// History			: First edition 						2013.03.22 Y.Shigeoka
//********************************************************************************
void	SetGcf_OFILM( unsigned char	UcSetNum_OFILM )
{
	
	/* Zoom Step */
	if(UcSetNum_OFILM > (COEFTBL_OFILM - 1))
		UcSetNum_OFILM = (COEFTBL_OFILM -1) ;			/* \8F\E3\8C\C0\82\F0COEFTBL_OFILM-1\82ɐݒ肷\82\E9 */

	UlH1Coefval_OFILM	= ClDiCof_OFILM[ UcSetNum_OFILM ] ;
		
	// Zoom Value Setting
	RamWrite32A_Ofilm( gxh1c_OFILM, UlH1Coefval_OFILM ) ;		/* 0x1012 */
	RamWrite32A_Ofilm( gyh1c_OFILM, UlH1Coefval_OFILM ) ;		/* 0x1112 */

#ifdef H1COEF_CHANGER_OFILM
		SetH1cMod_OFILM( UcSetNum_OFILM ) ;							/* Re-setting */
#endif

}

#ifdef H1COEF_CHANGER_OFILM
//********************************************************************************
// Function Name 	: SetH1cMod_OFILM
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set H1C coefficient Level chang Function
// History			: First edition 						2013.04.18 Y.Shigeoka
//********************************************************************************
void	SetH1cMod_OFILM( unsigned char	UcSetNum_OFILM )
{
	
	switch( UcSetNum_OFILM ){
	case ( ACTMODE_OFILM ):				// initial 
		IniPtMovMod_OFILM( OFF_OFILM ) ;							// Pan/Tilt setting (Still)
		
		/* enable setting */
		/* Zoom Step */
		UlH1Coefval_OFILM	= ClDiCof_OFILM[ 0 ] ;
			
		UcH1LvlMod_OFILM = 0 ;
		
		// Limit value Value Setting
		RamWrite32A_Ofilm( gxlmt6L_OFILM, MINLMT_OFILM ) ;		/* 0x102D L-Limit */
		RamWrite32A_Ofilm( gxlmt6H_OFILM, MAXLMT_OFILM ) ;		/* 0x102E H-Limit */

		RamWrite32A_Ofilm( gylmt6L_OFILM, MINLMT_OFILM ) ;		/* 0x112D L-Limit */
		RamWrite32A_Ofilm( gylmt6H_OFILM, MAXLMT_OFILM ) ;		/* 0x112E H-Limit */

		RamWrite32A_Ofilm( gxhc_tmp_OFILM, 	UlH1Coefval_OFILM ) ;	/* 0x100E Base Coef */
		RamWrite32A_Ofilm( gxmg_OFILM, 		CHGCOEF_OFILM ) ;		/* 0x10AA Change coefficient gain */

		RamWrite32A_Ofilm( gyhc_tmp_OFILM, 	UlH1Coefval_OFILM ) ;	/* 0x110E Base Coef */
		RamWrite32A_Ofilm( gymg_OFILM, 		CHGCOEF_OFILM ) ;		/* 0x11AA Change coefficient gain */
		
		RegWriteA_Ofilm( WG_HCHR_OFILM, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON_OFILM
		break ;
		
	case( S2MODE_OFILM ):				// cancel lvl change mode 
		RegWriteA_Ofilm( WG_HCHR_OFILM, 0x10 ) ;			// 0x011B	GmHChrOn[1]=0 Sw OFF_OFILM
		break ;
		
	case( MOVMODE_OFILM ):			// Movie mode 
		IniPtMovMod_OFILM( ON_OFILM ) ;							// Pan/Tilt setting (Movie)
		
		RamWrite32A_Ofilm( gxlmt6L_OFILM, MINLMT_MOV_OFILM ) ;	/* 0x102D L-Limit */
		RamWrite32A_Ofilm( gylmt6L_OFILM, MINLMT_MOV_OFILM ) ;	/* 0x112D L-Limit */

		RamWrite32A_Ofilm( gxmg_OFILM, CHGCOEF_MOV_OFILM ) ;		/* 0x10AA Change coefficient gain */
		RamWrite32A_Ofilm( gymg_OFILM, CHGCOEF_MOV_OFILM ) ;		/* 0x11AA Change coefficient gain */
			
		RamWrite32A_Ofilm( gxhc_tmp_OFILM, UlH1Coefval_OFILM ) ;		/* 0x100E Base Coef */
		RamWrite32A_Ofilm( gyhc_tmp_OFILM, UlH1Coefval_OFILM ) ;		/* 0x110E Base Coef */
		
		RegWriteA_Ofilm( WG_HCHR_OFILM, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON_OFILM
		break ;

	default :
		IniPtMovMod_OFILM( OFF_OFILM ) ;							// Pan/Tilt setting (Still)
		
		UcH1LvlMod_OFILM = UcSetNum_OFILM ;
			
		RamWrite32A_Ofilm( gxlmt6L_OFILM, MINLMT_OFILM ) ;		/* 0x102D L-Limit */
		RamWrite32A_Ofilm( gylmt6L_OFILM, MINLMT_OFILM ) ;		/* 0x112D L-Limit */
		
		RamWrite32A_Ofilm( gxmg_OFILM, 	CHGCOEF_OFILM ) ;			/* 0x10AA Change coefficient gain */
		RamWrite32A_Ofilm( gymg_OFILM, 	CHGCOEF_OFILM ) ;			/* 0x11AA Change coefficient gain */
			
		RamWrite32A_Ofilm( gxhc_tmp_OFILM, UlH1Coefval_OFILM ) ;		/* 0x100E Base Coef */
		RamWrite32A_Ofilm( gyhc_tmp_OFILM, UlH1Coefval_OFILM ) ;		/* 0x110E Base Coef */
		
		RegWriteA_Ofilm( WG_HCHR_OFILM, 0x12 ) ;			// 0x011B	GmHChrOn[1]=1 Sw ON_OFILM
		break ;
	}
}
#endif

//********************************************************************************
// Function Name 	: RdFwVr_OFILM
// Retun Value		: Firmware version
// Argment Value	: NON
// Explanation		: Read Fw Version Function
// History			: First edition 						2013.05.07 Y.Shigeoka
//********************************************************************************
unsigned short	RdFwVr_OFILM( void )
{
	unsigned short	UsVerVal_OFILM ;
	
	UsVerVal_OFILM = (unsigned short)((MDL_VER_OFILM << 8) | FW_VER_OFILM ) ;
	return( UsVerVal_OFILM ) ;
}

#ifdef	ACCEPTANCE_OFILM
//********************************************************************************
// Function Name 	: RunHea_OFILM
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Hall Examination of Acceptance
// History			: First edition 						2014.02.26 Y.Shigeoka
//********************************************************************************
unsigned char	RunHea_OFILM( void )
{
	unsigned char 	UcRst_OFILM ;
	
	UcRst_OFILM = EXE_END_OFILM ;
	UcRst_OFILM |= TstActMov_OFILM( X_DIR_OFILM) ;
	UcRst_OFILM |= TstActMov_OFILM( Y_DIR_OFILM) ;
	
TRACE("UcRst_OFILM = %02x\n", UcRst_OFILM ) ;
	
	return( UcRst_OFILM ) ;
}
unsigned char	TstActMov_OFILM( unsigned char UcDirSel_OFILM )
{
	unsigned char UcRsltSts_OFILM;
	unsigned short	UsMsppVal_OFILM ;

	MesFil_OFILM( NOISE_OFILM ) ;					// \91\AA\92\E8\97p\83t\83B\83\8B\83^\81[\82\F0\90ݒ肷\82\E9\81B

	if ( !UcDirSel_OFILM ) {
		RamWrite32A_Ofilm( sxsin_OFILM , ACT_CHK_LVL_OFILM );												// 0x10D5
		RamWrite32A_Ofilm( sysin_OFILM , 0x000000 );												// 0x11D5
		SetSinWavePara_OFILM( 0x05 , XACTTEST_OFILM ); 
	}else{
		RamWrite32A_Ofilm( sxsin_OFILM , 0x000000 );												// 0x10D5
		RamWrite32A_Ofilm( sysin_OFILM , ACT_CHK_LVL_OFILM );												// 0x11D5
		SetSinWavePara_OFILM( 0x05 , YACTTEST_OFILM ); 
	}

	if ( !UcDirSel_OFILM ) {					// AXIS X
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )SXINZ1_OFILM ) ;							/* 0x0194	*/
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( SXINZ1_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
	} else {							// AXIS Y
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM,  ( unsigned char )SYINZ1_OFILM ) ;							/* 0x0194	*/
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM,  ( unsigned char )(( SYINZ1_OFILM >> 8 ) & 0x0001 ) ) ;		/* 0x0195	*/
	}

	RegWriteA_Ofilm( WC_MESSINMODE_OFILM, 0x00 ) ;			// 0x0191	0 cross
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM, 0x00 ) ;				// 0x0193
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM, 0x02 ) ;				// 0x0192	2 Times Measure
	RamWrite32A_Ofilm( msmean_OFILM	, 0x3F000000 );				// 0x10AE	1/CmMesLoop[15:0]/2
	RegWriteA_Ofilm( WC_MESABS_OFILM, 0x00 ) ;				// 0x0198	none ABS
	BsyWit_OFILM( WC_MESMODE_OFILM, 0x02 ) ;				// 0x0190		Sine wave Measure

	RamAccFixMod_OFILM( ON_OFILM ) ;							// Fix mode
	RamReadA_Ofilm( MSPP1AV_OFILM, &UsMsppVal_OFILM ) ;
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// Float mode

	if ( !UcDirSel_OFILM ) {					// AXIS X
		SetSinWavePara_OFILM( 0x00 , XACTTEST_OFILM ); 	/* STOP */
	}else{
		SetSinWavePara_OFILM( 0x00 , YACTTEST_OFILM ); 	/* STOP */
	}

TRACE(" DIR = %04x, PP = %04x, ", UcDirSel_OFILM, UsMsppVal_OFILM ) ;

	
	UcRsltSts_OFILM = EXE_END_OFILM ;
	if( UsMsppVal_OFILM > ACT_THR_OFILM ){
		if ( !UcDirSel_OFILM ) {					// AXIS X
			UcRsltSts_OFILM = EXE_HXMVER_OFILM ;
		}else{								// AXIS Y
			UcRsltSts_OFILM = EXE_HYMVER_OFILM ;
		}
	}
	
	return( UcRsltSts_OFILM ) ;
}


//********************************************************************************
// Function Name 	: RunGea_OFILM
// Retun Value		: Result
// Argment Value	: NON
// Explanation		: Gyro Examination of Acceptance
// History			: First edition 						2014.02.13 T.Tokoro
//********************************************************************************
unsigned char	RunGea_OFILM( void )
{
	unsigned char 	UcRst_OFILM, UcCnt_OFILM, UcXLowCnt_OFILM, UcYLowCnt_OFILM, UcXHigCnt_OFILM, UcYHigCnt_OFILM ;
	unsigned short	UsGxoVal_OFILM[10], UsGyoVal_OFILM[10], UsDif_OFILM;
	
	UcRst_OFILM = EXE_END_OFILM ;
	UcXLowCnt_OFILM = UcYLowCnt_OFILM = UcXHigCnt_OFILM = UcYHigCnt_OFILM = 0 ;
	
	MesFil_OFILM( THROUGH_OFILM ) ;				// \91\AA\92\E8\97p\83t\83B\83\8B\83^\81[\82\F0\90ݒ肷\82\E9\81B
	
	for( UcCnt_OFILM = 0 ; UcCnt_OFILM < 10 ; UcCnt_OFILM++ )
	{
		// X
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM, 0x00 ) ;		// 0x0194
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM, 0x00 ) ;		// 0x0195
		ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );							// Measure Filter RAM Clear
		UsGxoVal_OFILM[UcCnt_OFILM] = (unsigned short)GenMes_OFILM( AD2Z_OFILM, 0 );	// 64\89\F1\82̕\BD\8Bϒl\91\AA\92\E8	GYRMON1(0x1110) <- GXADZ(0x144A)
		
		// Y
		RegWriteA_Ofilm( WC_MES1ADD0_OFILM, 0x00 ) ;		// 0x0194
		RegWriteA_Ofilm( WC_MES1ADD1_OFILM, 0x00 ) ;		// 0x0195
		ClrGyr_OFILM( 0x1000 , CLR_FRAM1_OFILM );							// Measure Filter RAM Clear
		UsGyoVal_OFILM[UcCnt_OFILM] = (unsigned short)GenMes_OFILM( AD3Z_OFILM, 0 );	// 64\89\F1\82̕\BD\8Bϒl\91\AA\92\E8	GYRMON2(0x1111) <- GYADZ(0x14CA)
		
TRACE("UcCnt_OFILM = %02x, UsGxoVal_OFILM[UcCnt_OFILM] = %04x\n", UcCnt_OFILM, UsGxoVal_OFILM[UcCnt_OFILM] ) ;
TRACE("UcCnt_OFILM = %02x, UsGyoVal_OFILM[UcCnt_OFILM] = %04x\n", UcCnt_OFILM, UsGyoVal_OFILM[UcCnt_OFILM] ) ;
		
//		WitTim_Ofilm( 100 ) ;							// Wait 100ms
		
		if( UcCnt_OFILM > 0 )
		{
			if ( (short)UsGxoVal_OFILM[0] > (short)UsGxoVal_OFILM[UcCnt_OFILM] ) {
				UsDif_OFILM = (unsigned short)((short)UsGxoVal_OFILM[0] - (short)UsGxoVal_OFILM[UcCnt_OFILM]) ;
			} else {
				UsDif_OFILM = (unsigned short)((short)UsGxoVal_OFILM[UcCnt_OFILM] - (short)UsGxoVal_OFILM[0]) ;
			}
			
			if( UsDif_OFILM > GEA_DIF_HIG_OFILM ) {
				//UcRst_OFILM = UcRst_OFILM | EXE_GXABOVE_OFILM ;
				UcXHigCnt_OFILM ++ ;
			}
			if( UsDif_OFILM < GEA_DIF_LOW_OFILM ) {
				//UcRst_OFILM = UcRst_OFILM | EXE_GXBELOW_OFILM ;
				UcXLowCnt_OFILM ++ ;
			}
			
			if ( (short)UsGyoVal_OFILM[0] > (short)UsGyoVal_OFILM[UcCnt_OFILM] ) {
				UsDif_OFILM = (unsigned short)((short)UsGyoVal_OFILM[0] - (short)UsGyoVal_OFILM[UcCnt_OFILM]) ;
			} else {
				UsDif_OFILM = (unsigned short)((short)UsGyoVal_OFILM[UcCnt_OFILM] - (short)UsGyoVal_OFILM[0]) ;
			}
			
			if( UsDif_OFILM > GEA_DIF_HIG_OFILM ) {
				//UcRst_OFILM = UcRst_OFILM | EXE_GYABOVE_OFILM ;
				UcYHigCnt_OFILM ++ ;
			}
			if( UsDif_OFILM < GEA_DIF_LOW_OFILM ) {
				//UcRst_OFILM = UcRst_OFILM | EXE_GYBELOW_OFILM ;
				UcYLowCnt_OFILM ++ ;
			}
		}
	}
	
	if( UcXHigCnt_OFILM >= 1 ) {
		UcRst_OFILM = UcRst_OFILM | EXE_GXABOVE_OFILM ;
	}
	if( UcXLowCnt_OFILM > 8 ) {
		UcRst_OFILM = UcRst_OFILM | EXE_GXBELOW_OFILM ;
	}
	
	if( UcYHigCnt_OFILM >= 1 ) {
		UcRst_OFILM = UcRst_OFILM | EXE_GYABOVE_OFILM ;
	}
	if( UcYLowCnt_OFILM > 8 ) {
		UcRst_OFILM = UcRst_OFILM | EXE_GYBELOW_OFILM ;
	}
	
TRACE("UcRst_OFILM = %02x\n", UcRst_OFILM ) ;
	
	return( UcRst_OFILM ) ;
}
#endif	//ACCEPTANCE_OFILM


//********************************************************************************
// Function Name 	: CmdRdChk_OFILM
// Retun Value		: 1 : ERROR
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2043.02.27 K.abe
//********************************************************************************

unsigned char CmdRdChk_OFILM( void )
{
	unsigned char UcTestRD_OFILM;
	unsigned char UcCount_OFILM;
	
	for(UcCount_OFILM=0; UcCount_OFILM < READ_COUNT_NUM_OFILM; UcCount_OFILM++){
		RegReadA_Ofilm( TESTRD_OFILM ,	&UcTestRD_OFILM );					// 0x027F
		if( UcTestRD_OFILM == 0xAC){
			return(0);
		}
	}
	//TRACE("***** Command Line Error !! Can't Read Data *****\n" ) ;
	//TRACE_ERROR();
	return(1);
}

