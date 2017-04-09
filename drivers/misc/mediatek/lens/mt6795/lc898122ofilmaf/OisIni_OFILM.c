//********************************************************************************
//
//		<< LC898122 Evaluation Soft >>
//		Program Name	: OisIni.c
//		Design			: Y.Yamada
//		History			: LC898122 						2013.01.09 Y.Shigeoka
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISINI_OFILM

//#include	"Main.h"
//#include	"Cmd.h"
#include	"Ois_OFILM.h"
#ifdef INIT_FAST_OFILM
#include	"OisFil_OFILM.h"
#else
#include	"OisFil_org_OFILM.h"
#endif
#include	"OisDef_OFILM.h"

//**************************
//	Local Function Prottype	
//**************************
void	IniClk_OFILM( void ) ;		// Clock Setting
void	IniIop_OFILM( void ) ;		// I/O Port Initial Setting
void	IniMon_OFILM( void ) ;		// Monitor & Other Initial Setting
void	IniSrv_OFILM( void ) ;		// Servo Register Initial Setting
void	IniGyr_OFILM( void ) ;		// Gyro Filter Register Initial Setting
void	IniFil_OFILM( void ) ;		// Gyro Filter Initial Parameter Setting
void	IniAdj_OFILM( void ) ;		// Adjust Fix Value Setting
void	IniCmd_OFILM( void ) ;		// Command Execute Process Initial
void	IniDgy_OFILM( void ) ;		// Digital Gyro Initial Setting
void	IniAf_OFILM( void ) ;			// Open AF Initial Setting
void	IniPtAve_OFILM( void ) ;		// Average setting
#define TRACE(x...) 


//********************************************************************************
// Function Name 	: IniSet_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSet_OFILM( void )
{
	// Command Execute Process Initial
TRACE("Initial CMD \n") ;
	IniCmd_OFILM() ;
	// Clock Setting
TRACE("Initial CLK \n") ;
	IniClk_OFILM() ;
	// I/O Port Initial Setting
TRACE("Initial IOP \n") ;
	IniIop_OFILM() ;
	// DigitalGyro Initial Setting
TRACE("Initial DGY \n") ;
	IniDgy_OFILM() ;
	// Monitor & Other Initial Setting
TRACE("Initial MON \n") ;
	IniMon_OFILM() ;
	// Servo Initial Setting
TRACE("Initial SRV \n") ;
	IniSrv_OFILM() ;
	// Gyro Filter Initial Setting
TRACE("Initial GYR \n") ;
	IniGyr_OFILM() ;
	// Gyro Filter Initial Setting
TRACE("Initial GFL \n") ;
	IniFil_OFILM() ;
	// Adjust Fix Value Setting
TRACE("Initial ADJ \n") ;
	IniAdj_OFILM() ;

}

//********************************************************************************
// Function Name 	: IniSetAf_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial AF Setting Function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniSetAf_OFILM( void )
{
	// Command Execute Process Initial
TRACE("Initial CMD \n") ;
	IniCmd_OFILM() ;
	// Clock Setting
TRACE("Initial CLK \n") ;
	IniClk_OFILM() ;
	// AF Initial Setting
TRACE("Initial AF \n") ;
	IniAf_OFILM() ;

}



//********************************************************************************
// Function Name 	: IniClk_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniClk_OFILM( void )
{
	ChkCvr_OFILM() ;								/* Read Cver */
	
	/*OSC Enables*/
	UcOscAdjFlg_OFILM	= 0 ;					// Osc adj flag 
	
#ifdef	DEF_SET_OFILM
	/*OSC ENABLE*/
	RegWriteA_Ofilm( OSCSTOP_OFILM,		0x00 ) ;			// 0x0256
	RegWriteA_Ofilm( OSCSET_OFILM,		0x90 ) ;			// 0x0257	OSC ini
	RegWriteA_Ofilm( OSCCNTEN_OFILM,	0x00 ) ;			// 0x0258	OSC Cnt disable
#endif
	/*Clock Enables*/
	RegWriteA_Ofilm( CLKON_OFILM,		0x1F ) ;			// 0x020B

#ifdef	USE_EXTCLK_ALL_OFILM
	RegWriteA_Ofilm( CLKSEL_OFILM,		0x07 ) ;			// 0x020C	All
#else
 #ifdef	USE_EXTCLK_PWM_OFILM
	RegWriteA_Ofilm( CLKSEL_OFILM,		0x01 ) ;			// 0x020C	only PWM
 #else
  #ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( CLKSEL_OFILM,		0x00 ) ;			// 0x020C	
  #endif
 #endif
#endif
	
#ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
	RegWriteA_Ofilm( PWMDIV_OFILM,		0x00 ) ;			// 0x0210	24MHz/1
	RegWriteA_Ofilm( SRVDIV_OFILM,		0x00 ) ;			// 0x0211	24MHz/1
	RegWriteA_Ofilm( GIFDIV_OFILM,		0x02 ) ;			// 0x0212	24MHz/2 = 12MHz
	RegWriteA_Ofilm( AFPWMDIV_OFILM,	0x00 ) ;			// 0x0213	24MHz/1 = 24MHz
	RegWriteA_Ofilm( OPAFDIV_OFILM,		0x02 ) ;			// 0x0214	24MHz/2 = 12MHz
#else
 #ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( PWMDIV_OFILM,		0x00 ) ;			// 0x0210	48MHz/1
	RegWriteA_Ofilm( SRVDIV_OFILM,		0x00 ) ;			// 0x0211	48MHz/1
	RegWriteA_Ofilm( GIFDIV_OFILM,		0x03 ) ;			// 0x0212	48MHz/3 = 16MHz
  #ifdef	AF_PWMMODE_OFILM
	RegWriteA_Ofilm( AFPWMDIV_OFILM,	0x00 ) ;			// 0x0213	48MHz/1
  #else
	RegWriteA_Ofilm( AFPWMDIV_OFILM,	0x02 ) ;			// 0x0213	48MHz/2 = 24MHz
  #endif
	RegWriteA_Ofilm( OPAFDIV_OFILM,		0x04 ) ;			// 0x0214	48MHz/4 = 12MHz
 #endif
#endif
}



//********************************************************************************
// Function Name 	: IniIop_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniIop_OFILM( void )
{
#ifdef	DEF_SET_OFILM
	/*set IOP direction*/
	RegWriteA_Ofilm( P0LEV_OFILM,		0x00 ) ;	// 0x0220	[ - 	| - 	| WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA_Ofilm( P0DIR_OFILM,		0x00 ) ;	// 0x0221	[ - 	| - 	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	/*set pull up/down*/
	RegWriteA_Ofilm( P0PON_OFILM,		0x0F ) ;	// 0x0222	[ -    | -	  | PON5 | PON4 ][ PON3  | PON2 | PON1 | PON0 ]
	RegWriteA_Ofilm( P0PUD_OFILM,		0x0F ) ;	// 0x0223	[ -    | -	  | PUD5 | PUD4 ][ PUD3  | PUD2 | PUD1 | PUD0 ]
#endif
	/*select IOP signal*/
#ifdef	USE_3WIRE_DGYRO_OFILM
	RegWriteA_Ofilm( IOP1SEL_OFILM,		0x02 ); 	// 0x0231	IOP1 : IOP1
#else
	RegWriteA_Ofilm( IOP1SEL_OFILM,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
#endif
#ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( IOP0SEL_OFILM,		0x02 ); 	// 0x0230	IOP0 : IOP0
	RegWriteA_Ofilm( IOP2SEL_OFILM,		0x02 ); 	// 0x0232	IOP2 : IOP2
	RegWriteA_Ofilm( IOP3SEL_OFILM,		0x00 ); 	// 0x0233	IOP3 : DGDATAOUT
	RegWriteA_Ofilm( IOP4SEL_OFILM,		0x00 ); 	// 0x0234	IOP4 : DGSCLK
	RegWriteA_Ofilm( IOP5SEL_OFILM,		0x00 ); 	// 0x0235	IOP5 : DGSSB
	RegWriteA_Ofilm( DGINSEL_OFILM,		0x00 ); 	// 0x0236	DGDATAIN 0:IOP1 1:IOP2
	RegWriteA_Ofilm( I2CSEL_OFILM,		0x00 );		// 0x0248	I2C noise reduction ON
	RegWriteA_Ofilm( DLMODE_OFILM,		0x00 );		// 0x0249	Download OFF_OFILM
#endif
	
}

//********************************************************************************
// Function Name 	: IniDgy_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniDgy_OFILM( void )
{
 #ifdef USE_INVENSENSE_OFILM
	unsigned char	UcGrini_OFILM ;
 #endif
	
	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
 #ifdef USE_3WIRE_DGYRO_OFILM
	RegWriteA_Ofilm( SPIM_OFILM 	, 0x00 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #else
	RegWriteA_Ofilm( SPIM_OFILM 	, 0x01 );							// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #endif
															//				DGSPI4	0: 3-wire SPI, 1: 4-wire SPI

	/*Set to Command Mode*/
	RegWriteA_Ofilm( GRSEL_OFILM	, 0x01 );							// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA_Ofilm( GRINI_OFILM	, 0x80 );							// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]

 #ifdef USE_INVENSENSE_OFILM

	RegReadA_Ofilm( GRINI_OFILM	, &UcGrini_OFILM );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]
	RegWriteA_Ofilm( GRINI_OFILM, ( UcGrini_OFILM | SLOWMODE_OFILM) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]
	
	RegWriteA_Ofilm( GRADR0_OFILM,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA_Ofilm( GSETDT_OFILM,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_OFILM( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegWriteA_Ofilm( GRADR0_OFILM,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA_Ofilm( GSETDT_OFILM,	( FS_SEL_OFILM << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit_OFILM( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegReadA_Ofilm( GRINI_OFILM	, &UcGrini_OFILM );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]
	RegWriteA_Ofilm( GRINI_OFILM, ( UcGrini_OFILM & ~SLOWMODE_OFILM) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]

 #endif
	
	RegWriteA_Ofilm( RDSEL_OFILM,	0x7C ) ;				// 0x028B	RDSEL_OFILM(Data1 and 2 for continuos mode)
	
	GyOutSignal_OFILM() ;
	

}


//********************************************************************************
// Function Name 	: IniMon_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniMon_OFILM( void )
{
	RegWriteA_Ofilm( PWMMONA_OFILM, 0x00 ) ;				// 0x0030	0:off
	
	RegWriteA_Ofilm( MONSELA_OFILM, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA_Ofilm( MONSELB_OFILM, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA_Ofilm( MONSELC_OFILM, 0x00 ) ;				// 0x0272	
	RegWriteA_Ofilm( MONSELD_OFILM, 0x00 ) ;				// 0x0273	

	// Monitor Circuit
	RegWriteA_Ofilm( WC_PINMON1_OFILM,	0x00 ) ;			// 0x01C0		Filter Monitor
	RegWriteA_Ofilm( WC_PINMON2_OFILM,	0x00 ) ;			// 0x01C1		
	RegWriteA_Ofilm( WC_PINMON3_OFILM,	0x00 ) ;			// 0x01C2		
	RegWriteA_Ofilm( WC_PINMON4_OFILM,	0x00 ) ;			// 0x01C3		
	/* Delay Monitor */
	RegWriteA_Ofilm( WC_DLYMON11_OFILM,	0x04 ) ;			// 0x01C5		DlyMonAdd1[10:8]
	RegWriteA_Ofilm( WC_DLYMON10_OFILM,	0x40 ) ;			// 0x01C4		DlyMonAdd1[ 7:0]
	RegWriteA_Ofilm( WC_DLYMON21_OFILM,	0x04 ) ;			// 0x01C7		DlyMonAdd2[10:8]
	RegWriteA_Ofilm( WC_DLYMON20_OFILM,	0xC0 ) ;			// 0x01C6		DlyMonAdd2[ 7:0]
	RegWriteA_Ofilm( WC_DLYMON31_OFILM,	0x00 ) ;			// 0x01C9		DlyMonAdd3[10:8]
	RegWriteA_Ofilm( WC_DLYMON30_OFILM,	0x00 ) ;			// 0x01C8		DlyMonAdd3[ 7:0]
	RegWriteA_Ofilm( WC_DLYMON41_OFILM,	0x00 ) ;			// 0x01CB		DlyMonAdd4[10:8]
	RegWriteA_Ofilm( WC_DLYMON40_OFILM,	0x00 ) ;			// 0x01CA		DlyMonAdd4[ 7:0]

/* Monitor */
	RegWriteA_Ofilm( PWMMONA_OFILM, 0x80 ) ;				// 0x0030	1:on 
//	RegWriteA_Ofilm( IOP0SEL_OFILM,		0x01 ); 			// 0x0230	IOP0 : MONA
/**/


}

//********************************************************************************
// Function Name 	: IniSrv_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniSrv_OFILM( void )
{
	unsigned char	UcStbb0_OFILM ;

	UcPwmMod_OFILM = INIT_PWMMODE_OFILM ;					// Driver output mode

	RegWriteA_Ofilm( WC_EQON_OFILM,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA_Ofilm( WC_RAMINITON_OFILM,0x00 ) ;				// 0x0102		
	ClrGyr_OFILM( 0x0000 , CLR_ALL_RAM_OFILM );					// All Clear
	
	RegWriteA_Ofilm( WH_EQSWX_OFILM,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA_Ofilm( WH_EQSWY_OFILM,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// 32bit Float mode
	
	/* Monitor Gain */
	RamWrite32A_Ofilm( dm1g_OFILM, 0x3F800000 ) ;			// 0x109A
	RamWrite32A_Ofilm( dm2g_OFILM, 0x3F800000 ) ;			// 0x109B
	RamWrite32A_Ofilm( dm3g_OFILM, 0x3F800000 ) ;			// 0x119A
	RamWrite32A_Ofilm( dm4g_OFILM, 0x3F800000 ) ;			// 0x119B
	
	/* Hall output limitter */
	RamWrite32A_Ofilm( sxlmta1_OFILM,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
	RamWrite32A_Ofilm( sylmta1_OFILM,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	
	/* Emargency Stop */
	RegWriteA_Ofilm( WH_EMGSTPON_OFILM,	0x00 ) ;				// 0x0178		Emargency Stop OFF_OFILM
	RegWriteA_Ofilm( WH_EMGSTPTMR_OFILM,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A_Ofilm( sxemglev_OFILM,   0x3F800000 ) ;			// 0x10EC		Hall X Emargency threshold
	RamWrite32A_Ofilm( syemglev_OFILM,   0x3F800000 ) ;			// 0x11EC		Hall Y Emargency threshold
	
	/* Hall Servo smoothing */
	RegWriteA_Ofilm( WH_SMTSRVON_OFILM,	0x00 ) ;				// 0x017C		Smooth Servo OFF_OFILM
#ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
	RegWriteA_Ofilm( WH_SMTSRVSMP_OFILM,0x03 ) ;				// 0x017D		2.7ms=2^03/11.718kHz
	RegWriteA_Ofilm( WH_SMTTMR_OFILM,	0x00 ) ;				// 0x017E		1.3ms=(0+1)*16/11.718kHz
#else
	RegWriteA_Ofilm( WH_SMTSRVSMP_OFILM,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
	RegWriteA_Ofilm( WH_SMTTMR_OFILM,	0x01 ) ;				// 0x017E		1.3ms=(1+1)*16/23.4375kHz
#endif
	
	RamWrite32A_Ofilm( sxsmtav_OFILM,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A_Ofilm( sysmtav_OFILM,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A_Ofilm( sxsmtstp_OFILM,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A_Ofilm( sysmtstp_OFILM,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA_Ofilm( WH_HOFCON_OFILM,	0x11 ) ;				// 0x0174		OUT 3x3
	
	/* Front */
	RamWrite32A_Ofilm( sxiexp3_OFILM,   A3_IEXP3_OFILM ) ;			// 0x10BA		
	RamWrite32A_Ofilm( sxiexp2_OFILM,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A_Ofilm( sxiexp1_OFILM,   A1_IEXP1_OFILM ) ;			// 0x10BC		
	RamWrite32A_Ofilm( sxiexp0_OFILM,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A_Ofilm( sxiexp_OFILM,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A_Ofilm( syiexp3_OFILM,   A3_IEXP3_OFILM ) ;			// 0x11BA		
	RamWrite32A_Ofilm( syiexp2_OFILM,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A_Ofilm( syiexp1_OFILM,   A1_IEXP1_OFILM ) ;			// 0x11BC		
	RamWrite32A_Ofilm( syiexp0_OFILM,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A_Ofilm( syiexp_OFILM,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A_Ofilm( sxoexp3_OFILM,   A3_IEXP3_OFILM ) ;			// 0x10FA		
	RamWrite32A_Ofilm( sxoexp2_OFILM,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A_Ofilm( sxoexp1_OFILM,   A1_IEXP1_OFILM ) ;			// 0x10FC		
	RamWrite32A_Ofilm( sxoexp0_OFILM,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A_Ofilm( sxoexp_OFILM,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A_Ofilm( syoexp3_OFILM,   A3_IEXP3_OFILM ) ;			// 0x11FA		
	RamWrite32A_Ofilm( syoexp2_OFILM,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A_Ofilm( syoexp1_OFILM,   A1_IEXP1_OFILM ) ;			// 0x11FC		
	RamWrite32A_Ofilm( syoexp0_OFILM,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A_Ofilm( syoexp_OFILM,    0x3F800000 ) ;			// 0x11FE		
	
	/* Sine wave */
#ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( WC_SINON_OFILM,	0x00 ) ;				// 0x0180		Sin Wave off
	RegWriteA_Ofilm( WC_SINFRQ0_OFILM,	0x00 ) ;				// 0x0181		
	RegWriteA_Ofilm( WC_SINFRQ1_OFILM,	0x60 ) ;				// 0x0182		
	RegWriteA_Ofilm( WC_SINPHSX_OFILM,	0x00 ) ;				// 0x0183		
	RegWriteA_Ofilm( WC_SINPHSY_OFILM,	0x20 ) ;				// 0x0184		
	
	/* AD over sampling */
	RegWriteA_Ofilm( WC_ADMODE_OFILM,	0x06 ) ;				// 0x0188		AD Over Sampling
	
	/* Measure mode */
	RegWriteA_Ofilm( WC_MESMODE_OFILM,		0x00 ) ;				// 0x0190		Measurement Mode
	RegWriteA_Ofilm( WC_MESSINMODE_OFILM,	0x00 ) ;				// 0x0191		
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM,		0x08 ) ;				// 0x0192		
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM,		0x02 ) ;				// 0x0193		
	RegWriteA_Ofilm( WC_MES1ADD0_OFILM,		0x00 ) ;				// 0x0194		
	RegWriteA_Ofilm( WC_MES1ADD1_OFILM,		0x00 ) ;				// 0x0195		
	RegWriteA_Ofilm( WC_MES2ADD0_OFILM,		0x00 ) ;				// 0x0196		
	RegWriteA_Ofilm( WC_MES2ADD1_OFILM,		0x00 ) ;				// 0x0197		
	RegWriteA_Ofilm( WC_MESABS_OFILM,		0x00 ) ;				// 0x0198		
	RegWriteA_Ofilm( WC_MESWAIT_OFILM,		0x00 ) ;				// 0x0199		
	
	/* auto measure */
	RegWriteA_Ofilm( WC_AMJMODE_OFILM,		0x00 ) ;				// 0x01A0		Automatic measurement mode
	
	RegWriteA_Ofilm( WC_AMJLOOP0_OFILM,		0x08 ) ;				// 0x01A2		Self-Aadjustment
	RegWriteA_Ofilm( WC_AMJLOOP1_OFILM,		0x02 ) ;				// 0x01A3		
	RegWriteA_Ofilm( WC_AMJIDL0_OFILM,		0x02 ) ;				// 0x01A4		
	RegWriteA_Ofilm( WC_AMJIDL1_OFILM,		0x00 ) ;				// 0x01A5		
	RegWriteA_Ofilm( WC_AMJ1ADD0_OFILM,		0x00 ) ;				// 0x01A6		
	RegWriteA_Ofilm( WC_AMJ1ADD1_OFILM,		0x00 ) ;				// 0x01A7		
	RegWriteA_Ofilm( WC_AMJ2ADD0_OFILM,		0x00 ) ;				// 0x01A8		
	RegWriteA_Ofilm( WC_AMJ2ADD1_OFILM,		0x00 ) ;				// 0x01A9		
	
	/* Data Pass */
	RegWriteA_Ofilm( WC_DPI1ADD0_OFILM,		0x00 ) ;				// 0x01B0		Data Pass
	RegWriteA_Ofilm( WC_DPI1ADD1_OFILM,		0x00 ) ;				// 0x01B1		
	RegWriteA_Ofilm( WC_DPI2ADD0_OFILM,		0x00 ) ;				// 0x01B2		
	RegWriteA_Ofilm( WC_DPI2ADD1_OFILM,		0x00 ) ;				// 0x01B3		
	RegWriteA_Ofilm( WC_DPI3ADD0_OFILM,		0x00 ) ;				// 0x01B4		
	RegWriteA_Ofilm( WC_DPI3ADD1_OFILM,		0x00 ) ;				// 0x01B5		
	RegWriteA_Ofilm( WC_DPI4ADD0_OFILM,		0x00 ) ;				// 0x01B6		
	RegWriteA_Ofilm( WC_DPI4ADD1_OFILM,		0x00 ) ;				// 0x01B7		
	RegWriteA_Ofilm( WC_DPO1ADD0_OFILM,		0x00 ) ;				// 0x01B8		Data Pass
	RegWriteA_Ofilm( WC_DPO1ADD1_OFILM,		0x00 ) ;				// 0x01B9		
	RegWriteA_Ofilm( WC_DPO2ADD0_OFILM,		0x00 ) ;				// 0x01BA		
	RegWriteA_Ofilm( WC_DPO2ADD1_OFILM,		0x00 ) ;				// 0x01BB		
	RegWriteA_Ofilm( WC_DPO3ADD0_OFILM,		0x00 ) ;				// 0x01BC		
	RegWriteA_Ofilm( WC_DPO3ADD1_OFILM,		0x00 ) ;				// 0x01BD		
	RegWriteA_Ofilm( WC_DPO4ADD0_OFILM,		0x00 ) ;				// 0x01BE		
	RegWriteA_Ofilm( WC_DPO4ADD1_OFILM,		0x00 ) ;				// 0x01BF		
	RegWriteA_Ofilm( WC_DPON_OFILM,			0x00 ) ;				// 0x0105		Data pass OFF_OFILM
	
	/* Interrupt Flag */
	RegWriteA_Ofilm( WC_INTMSK_OFILM,	0xFF ) ;				// 0x01CE		All Mask
	
#endif
	
	/* Ram Access */
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// 32bit float mode

	// PWM Signal Generate
	DrvSw_OFILM( OFF_OFILM ) ;									/* 0x0070	Drvier Block Ena=0 */
	RegWriteA_Ofilm( DRVFC2_OFILM	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30 ns
	RegWriteA_Ofilm( DRVSELX_OFILM	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA_Ofilm( DRVSELY_OFILM	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK_OFILM
 #ifdef	PWM_CAREER_TEST_OFILM
	RegWriteA_Ofilm( PWMFC_OFILM,		0x7C ) ;				// 0x0011	VREF, PWMFRQ=7:PWMCLK(EXCLK)/PWMPERIODX[5:2]=18MHz/4=4.5MHz, MODE0B, 11-bit Accuracy
 #else		//PWM_CAREER_TEST_OFILM
	if( UcCvrCod_OFILM == CVER122_OFILM ) {
		RegWriteA_Ofilm( PWMFC_OFILM,   0x2D ) ;					// 0x0011	VREF, PWMCLK/256, MODE0B, 12Bit Accuracy
	} else {
		RegWriteA_Ofilm( PWMFC_OFILM,   0x3D ) ;					// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
	}
 #endif	//PWM_CAREER_TEST_OFILM
#else
 #ifdef	PWM_CAREER_TEST_OFILM
	RegWriteA_Ofilm( PWMFC_OFILM,		0x78 ) ;				// 0x0011	VREF, PWMFRQ=7:PWMCLK(EXCLK)/PWMPERIODX[5:2]=18MHz/4=4.5MHz, MODE0S, 11-bit Accuracy  
 #else		//PWM_CAREER_TEST_OFILM
	if( UcCvrCod_OFILM == CVER122_OFILM ) {
		RegWriteA_Ofilm( PWMFC_OFILM,   0x29 ) ;					// 0x0011	VREF, PWMCLK/256, MODE0S, 12Bit Accuracy  
	} else {
		RegWriteA_Ofilm( PWMFC_OFILM,   0x39 ) ;					// 0x0011	VREF, PWMCLK/128, MODE0S, 12Bit Accuracy  
	}
 #endif	//PWM_CAREER_TEST_OFILM
#endif

#ifdef	USE_VH_SYNC_OFILM
	RegWriteA_Ofilm( STROBEFC_OFILM,	0x80 ) ;				// 0x001C	
	RegWriteA_Ofilm( STROBEDLYX_OFILM,	0x00 ) ;				// 0x001D	Delay
	RegWriteA_Ofilm( STROBEDLYY_OFILM,	0x00 ) ;				// 0x001E	Delay
#endif	//USE_VH_SYNC_OFILM

	RegWriteA_Ofilm( PWMA_OFILM,    0x00 ) ;					// 0x0010	PWM X/Y standby
	RegWriteA_Ofilm( PWMDLYX_OFILM,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA_Ofilm( PWMDLYY_OFILM,  0x04 ) ;					// 0x0013	Y Phase Delay Setting
	
#ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( DRVCH1SEL_OFILM,	0x00 ) ;				// 0x0005	OUT1/OUT2	X axis
	RegWriteA_Ofilm( DRVCH2SEL_OFILM,	0x00 ) ;				// 0x0006	OUT3/OUT4	Y axis
	
	RegWriteA_Ofilm( PWMDLYTIMX_OFILM,	0x00 ) ;				// 0x0014		PWM Timing
	RegWriteA_Ofilm( PWMDLYTIMY_OFILM,	0x00 ) ;				// 0x0015		PWM Timing
#endif
	
	if( UcCvrCod_OFILM == CVER122_OFILM ) {
#ifdef	PWM_CAREER_TEST_OFILM
		RegWriteA_Ofilm( PWMPERIODY_OFILM,	0xD0 ) ;				// 0x001A	11010000h --> PWMPERIODX[5:2] = 0100h = 4
		RegWriteA_Ofilm( PWMPERIODY2_OFILM,	0xD0 ) ;				// 0x001B	11010000h --> PWMPERIODY[5:2] = 0100h = 4
#else		//PWM_CAREER_TEST_OFILM
		RegWriteA_Ofilm( PWMPERIODY_OFILM,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODY2_OFILM,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#endif
	} else {
#ifdef	PWM_CAREER_TEST_OFILM
		RegWriteA_Ofilm( PWMPERIODX_OFILM,	0xF2 ) ;				// 0x0018		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODX2_OFILM,	0x00 ) ;				// 0x0019		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODY_OFILM,	0xF2 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODY2_OFILM,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#else		//PWM_CAREER_TEST_OFILM
		RegWriteA_Ofilm( PWMPERIODX_OFILM,	0x00 ) ;				// 0x0018		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODX2_OFILM,	0x00 ) ;				// 0x0019		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODY_OFILM,	0x00 ) ;				// 0x001A		PWM Carrier Freq
		RegWriteA_Ofilm( PWMPERIODY2_OFILM,	0x00 ) ;				// 0x001B		PWM Carrier Freq
#endif
	}
	
	/* Linear PWM circuit setting */
	RegWriteA_Ofilm( CVA_OFILM		, 0xC0 );			// 0x0020	Linear PWM mode enable

	if( UcCvrCod_OFILM == CVER122_OFILM ) {
		RegWriteA_Ofilm( CVFC_OFILM 	, 0x22 );			// 0x0021	
	}
	
#ifdef	PWM_BREAK_OFILM 
	RegWriteA_Ofilm( CVFC2_OFILM 	, 0x80 );			// 0x0022
#else
	RegWriteA_Ofilm( CVFC2_OFILM 	, 0x00 );			// 0x0022
#endif
	if( UcCvrCod_OFILM == CVER122_OFILM ) {
		RegWriteA_Ofilm( CVSMTHX_OFILM	, 0x00 );			// 0x0023	smooth off
		RegWriteA_Ofilm( CVSMTHY_OFILM	, 0x00 );			// 0x0024	smooth off
	}

	RegReadA_Ofilm( STBB0_OFILM 	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0_OFILM &= 0x80 ;
	RegWriteA_Ofilm( STBB0_OFILM, UcStbb0_OFILM ) ;			// 0x0250	OIS standby
	
}



//********************************************************************************
// Function Name 	: IniGyr_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
#ifdef GAIN_CONT_OFILM
  #define	TRI_LEVEL_OFILM		0x3A031280		/* 0.0005 */
  #define	TIMELOW_OFILM			0x50			/* */
  #define	TIMEHGH_OFILM			0x05			/* */
 #ifdef	USE_EXTCLK_ALL_OFILM	// 24MHz
  #define	TIMEBSE_OFILM			0x2F			/* 4.0ms */
 #else
  #define	TIMEBSE_OFILM			0x5D			/* 3.96ms */
 #endif
  #define	MONADR_OFILM			GXXFZ_OFILM
  #define	GANADR_OFILM			gxadj_OFILM
  #define	XMINGAIN_OFILM		0x00000000
  #define	XMAXGAIN_OFILM		0x3F800000
  #define	YMINGAIN_OFILM		0x00000000
  #define	YMAXGAIN_OFILM		0x3F800000
  #define	XSTEPUP_OFILM			0x38D1B717		/* 0.0001	 */
  #define	XSTEPDN_OFILM			0xBD4CCCCD		/* -0.05 	 */
  #define	YSTEPUP_OFILM			0x38D1B717		/* 0.0001	 */
  #define	YSTEPDN_OFILM			0xBD4CCCCD		/* -0.05 	 */
#endif


void	IniGyr_OFILM( void )
{
	
	
	/*Gyro Filter Setting*/
	RegWriteA_Ofilm( WG_EQSW_OFILM	, 0x03 );		// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	/*Gyro Filter Down Sampling*/
	
	RegWriteA_Ofilm( WG_SHTON_OFILM	, 0x10 );		// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
										//				CmShtOpe[1:0] 00: 							
#ifdef	DEF_SET_OFILM
	RegWriteA_Ofilm( WG_SHTDLYTMR_OFILM , 0x00 );	// 0x0117	 	Shutter Delay
	RegWriteA_Ofilm( WG_GADSMP_OFILM, 	  0x00 );	// 0x011C		Sampling timing
	RegWriteA_Ofilm( WG_HCHR_OFILM, 	  0x00 );	// 0x011B		H-filter limitter control not USE
	RegWriteA_Ofilm( WG_LMT3MOD_OFILM , 0x00 );		// 0x0118 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
										//				CmLmt3Mod	0: 
	RegWriteA_Ofilm( WG_VREFADD_OFILM , 0x12 );		// 0x0119	 	
#endif
	RegWriteA_Ofilm( WG_SHTMOD_OFILM , 0x06 );		// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A_Ofilm( gxlmt1H_OFILM, GYRLMT1H_OFILM ) ;			// 0x1028
	RamWrite32A_Ofilm( gylmt1H_OFILM, GYRLMT1H_OFILM ) ;			// 0x1128

	RamWrite32A_Ofilm( gxlmt3HS0_OFILM, GYRLMT3_S1_OFILM ) ;		// 0x1029
	RamWrite32A_Ofilm( gylmt3HS0_OFILM, GYRLMT3_S1_OFILM ) ;		// 0x1129
	
	RamWrite32A_Ofilm( gxlmt3HS1_OFILM, GYRLMT3_S2_OFILM ) ;		// 0x102A
	RamWrite32A_Ofilm( gylmt3HS1_OFILM, GYRLMT3_S2_OFILM ) ;		// 0x112A

	RamWrite32A_Ofilm( gylmt4HS0_OFILM, GYRLMT4_S1_OFILM ) ;		//0x112B	YLimiter4 High臒l0
	RamWrite32A_Ofilm( gxlmt4HS0_OFILM, GYRLMT4_S1_OFILM ) ;		//0x102B	XLimiter4 High臒l0
	
	RamWrite32A_Ofilm( gxlmt4HS1_OFILM, GYRLMT4_S2_OFILM ) ;		//0x102C	XLimiter4 High臒l1
	RamWrite32A_Ofilm( gylmt4HS1_OFILM, GYRLMT4_S2_OFILM ) ;		//0x112C	YLimiter4 High臒l1

	
	/* Pan/Tilt parameter */
	RegWriteA_Ofilm( WG_PANADDA_OFILM, 		0x12 );		// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA_Ofilm( WG_PANADDB_OFILM, 		0x09 );		// 0x0131	GXIZ/GYIZ Select
	
	 //Threshold
	RamWrite32A_Ofilm( SttxHis_OFILM, 	0x00000000 );			// 0x1226
	RamWrite32A_Ofilm( SttxaL_OFILM, 	0x00000000 );			// 0x109D
	RamWrite32A_Ofilm( SttxbL_OFILM, 	0x00000000 );			// 0x109E
	RamWrite32A_Ofilm( Sttx12aM_OFILM, 	GYRA12_MID_OFILM );	// 0x104F
	RamWrite32A_Ofilm( Sttx12aH_OFILM, 	GYRA12_HGH_OFILM );	// 0x105F
	RamWrite32A_Ofilm( Sttx12bM_OFILM, 	GYRB12_MID_OFILM );	// 0x106F
	RamWrite32A_Ofilm( Sttx12bH_OFILM, 	GYRB12_HGH_OFILM );	// 0x107F
	RamWrite32A_Ofilm( Sttx34aM_OFILM, 	GYRA34_MID_OFILM );	// 0x108F
	RamWrite32A_Ofilm( Sttx34aH_OFILM, 	GYRA34_HGH_OFILM );	// 0x109F
	RamWrite32A_Ofilm( Sttx34bM_OFILM, 	GYRB34_MID_OFILM );	// 0x10AF
	RamWrite32A_Ofilm( Sttx34bH_OFILM, 	GYRB34_HGH_OFILM );	// 0x10BF
	RamWrite32A_Ofilm( SttyaL_OFILM, 	0x00000000 );			// 0x119D
	RamWrite32A_Ofilm( SttybL_OFILM, 	0x00000000 );			// 0x119E
	RamWrite32A_Ofilm( Stty12aM_OFILM, 	GYRA12_MID_OFILM );	// 0x114F
	RamWrite32A_Ofilm( Stty12aH_OFILM, 	GYRA12_HGH_OFILM );	// 0x115F
	RamWrite32A_Ofilm( Stty12bM_OFILM, 	GYRB12_MID_OFILM );	// 0x116F
	RamWrite32A_Ofilm( Stty12bH_OFILM, 	GYRB12_HGH_OFILM );	// 0x117F
	RamWrite32A_Ofilm( Stty34aM_OFILM, 	GYRA34_MID_OFILM );	// 0x118F
	RamWrite32A_Ofilm( Stty34aH_OFILM, 	GYRA34_HGH_OFILM );	// 0x119F
	RamWrite32A_Ofilm( Stty34bM_OFILM, 	GYRB34_MID_OFILM );	// 0x11AF
	RamWrite32A_Ofilm( Stty34bH_OFILM, 	GYRB34_HGH_OFILM );	// 0x11BF
	
	// Pan level
	RegWriteA_Ofilm( WG_PANLEVABS_OFILM, 		0x00 );		// 0x0133
	
	// Average parameter are set IniAdj_OFILM

	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA_Ofilm( WG_PANSTT21JUG0_OFILM, 	0x00 );		// 0x0140
	RegWriteA_Ofilm( WG_PANSTT21JUG1_OFILM, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA_Ofilm( WG_PANSTT31JUG0_OFILM, 	0x00 );		// 0x0142
	RegWriteA_Ofilm( WG_PANSTT31JUG1_OFILM, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA_Ofilm( WG_PANSTT41JUG0_OFILM, 	0x01 );		// 0x0144
	RegWriteA_Ofilm( WG_PANSTT41JUG1_OFILM, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA_Ofilm( WG_PANSTT12JUG0_OFILM, 	0x00 );		// 0x0146
	RegWriteA_Ofilm( WG_PANSTT12JUG1_OFILM, 	0x07 );		// 0x0147
	// State 1 -> 3
	RegWriteA_Ofilm( WG_PANSTT13JUG0_OFILM, 	0x00 );		// 0x0148
	RegWriteA_Ofilm( WG_PANSTT13JUG1_OFILM, 	0x00 );		// 0x0149
	// State 2 -> 3
	RegWriteA_Ofilm( WG_PANSTT23JUG0_OFILM, 	0x11 );		// 0x014A
	RegWriteA_Ofilm( WG_PANSTT23JUG1_OFILM, 	0x00 );		// 0x014B
	// State 4 -> 3
	RegWriteA_Ofilm( WG_PANSTT43JUG0_OFILM, 	0x00 );		// 0x014C
	RegWriteA_Ofilm( WG_PANSTT43JUG1_OFILM, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA_Ofilm( WG_PANSTT34JUG0_OFILM, 	0x01 );		// 0x014E
	RegWriteA_Ofilm( WG_PANSTT34JUG1_OFILM, 	0x00 );		// 0x014F
	// State 2 -> 4
	RegWriteA_Ofilm( WG_PANSTT24JUG0_OFILM, 	0x00 );		// 0x0150
	RegWriteA_Ofilm( WG_PANSTT24JUG1_OFILM, 	0x00 );		// 0x0151
	// State 4 -> 2
	RegWriteA_Ofilm( WG_PANSTT42JUG0_OFILM, 	0x44 );		// 0x0152
	RegWriteA_Ofilm( WG_PANSTT42JUG1_OFILM, 	0x04 );		// 0x0153

	// State Timer
	RegWriteA_Ofilm( WG_PANSTT1LEVTMR_OFILM, 	0x00 );		// 0x015B
	RegWriteA_Ofilm( WG_PANSTT2LEVTMR_OFILM, 	0x00 );		// 0x015C
	RegWriteA_Ofilm( WG_PANSTT3LEVTMR_OFILM, 	0x00 );		// 0x015D
	RegWriteA_Ofilm( WG_PANSTT4LEVTMR_OFILM, 	0x03 );		// 0x015E
	
	// Control filter
	RegWriteA_Ofilm( WG_PANTRSON0_OFILM, 		0x11 );		// 0x0132	USE I12/iSTP/Gain-Filter
	
	// State Setting
	IniPtMovMod_OFILM( OFF_OFILM ) ;							// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA_Ofilm( WG_PANSTTSETILHLD_OFILM,	0x00 );		// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA_Ofilm( WG_PANSTT2TMR0_OFILM,	0x01 );		// 0x013C
	RegWriteA_Ofilm( WG_PANSTT2TMR1_OFILM,	0x00 );		// 0x013D	
	RegWriteA_Ofilm( WG_PANSTT4TMR0_OFILM,	0x02 );		// 0x013E
	RegWriteA_Ofilm( WG_PANSTT4TMR1_OFILM,	0x07 );		// 0x013F	
	
	RegWriteA_Ofilm( WG_PANSTTXXXTH_OFILM,	0x00 );		// 0x015A

#ifdef GAIN_CONT_OFILM
	RamWrite32A_Ofilm( gxlevlow_OFILM, TRI_LEVEL_OFILM );					// 0x10AE	Low Th
	RamWrite32A_Ofilm( gylevlow_OFILM, TRI_LEVEL_OFILM );					// 0x11AE	Low Th
	RamWrite32A_Ofilm( gxadjmin_OFILM, XMINGAIN_OFILM );					// 0x1094	Low gain
	RamWrite32A_Ofilm( gxadjmax_OFILM, XMAXGAIN_OFILM );					// 0x1095	Hgh gain
	RamWrite32A_Ofilm( gxadjdn_OFILM, XSTEPDN_OFILM );					// 0x1096	-step
	RamWrite32A_Ofilm( gxadjup_OFILM, XSTEPUP_OFILM );					// 0x1097	+step
	RamWrite32A_Ofilm( gyadjmin_OFILM, YMINGAIN_OFILM );					// 0x1194	Low gain
	RamWrite32A_Ofilm( gyadjmax_OFILM, YMAXGAIN_OFILM );					// 0x1195	Hgh gain
	RamWrite32A_Ofilm( gyadjdn_OFILM, YSTEPDN_OFILM );					// 0x1196	-step
	RamWrite32A_Ofilm( gyadjup_OFILM, YSTEPUP_OFILM );					// 0x1197	+step
	
	RegWriteA_Ofilm( WG_LEVADD_OFILM, (unsigned char)MONADR_OFILM );		// 0x0120	Input signal
	RegWriteA_Ofilm( WG_LEVTMR_OFILM, 		TIMEBSE_OFILM );				// 0x0123	Base Time
	RegWriteA_Ofilm( WG_LEVTMRLOW_OFILM, 	TIMELOW_OFILM );				// 0x0121	X Low Time
	RegWriteA_Ofilm( WG_LEVTMRHGH_OFILM, 	TIMEHGH_OFILM );				// 0x0122	X Hgh Time
	RegWriteA_Ofilm( WG_ADJGANADD_OFILM, (unsigned char)GANADR_OFILM );		// 0x0128	control address
	RegWriteA_Ofilm( WG_ADJGANGO_OFILM, 		0x00 );					// 0x0108	manual off

	/* exe function */
	AutoGainControlSw_OFILM( ON_OFILM ) ;							/* Auto Gain Control Mode OFF_OFILM */
#endif
	
}

extern int  camera_sensor_is_3m2;

//********************************************************************************
// Function Name 	: IniFil_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniFil_OFILM( void )
{
#ifdef INIT_FAST_OFILM
	unsigned char	UcAryId ;
	unsigned short	UsDatId, UsDatNum ;

	unsigned char	*pFilRegDat;
	unsigned char	*pFilReg;
	unsigned char	*pFilRamDat;
	unsigned char	*pFilRam;

	pFilRegDat	= (unsigned char *)CsFilRegDat_13M_OFILM;
	pFilReg		= (unsigned char *)CsFilReg_13M_OFILM;
	pFilRamDat	= (unsigned char *)CsFilRamDat_13M_OFILM;
	pFilRam		= (unsigned char *)CsFilRam_13M_OFILM;

	// Filter Registor Parameter Setting
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilReg[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilReg[ UcAryId ];
		SeqWriteA_Ofilm( ( unsigned char * )&pFilRegDat[ UsDatId ], UsDatNum ) ;
		UcAryId++ ;
		UsDatId	+= UsDatNum ;
	}

	// Filter X-axis Ram Parameter Setting	
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilRam[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilRam[ UcAryId ];
		SeqWriteA_Ofilm( ( unsigned char * )&pFilRamDat[ UsDatId ], UsDatNum ) ;
		UsDatId	+= UsDatNum ;
		UcAryId++ ;
	}
#else
 	unsigned short	UsAryId_OFILM ;

	// Filter Registor Parameter Setting
	UsAryId_OFILM	= 0 ;
	while( CsFilReg_OFILM[ UsAryId_OFILM ].UsRegAdd_OFILM != 0xFFFF )
	{
		RegWriteA_Ofilm( CsFilReg_OFILM[ UsAryId_OFILM ].UsRegAdd_OFILM, CsFilReg_OFILM[ UsAryId_OFILM ].UcRegDat_OFILM ) ;
		UsAryId_OFILM++ ;
	}

	// Filter Ram Parameter Setting
	UsAryId_OFILM	= 0 ;

	if(camera_sensor_is_3m2 == 1)
	{
		while( CsFilRam_3m2[ UsAryId_OFILM ].UsRamAdd_OFILM != 0xFFFF )
		{
			RamWrite32A_Ofilm( CsFilRam_3m2[ UsAryId_OFILM ].UsRamAdd_OFILM, CsFilRam_3m2[ UsAryId_OFILM ].UlRamDat_OFILM ) ;
			UsAryId_OFILM++ ;
		}
	}
	else
	{
		while( CsFilRam_OFILM[ UsAryId_OFILM ].UsRamAdd_OFILM != 0xFFFF )
		{
			RamWrite32A_Ofilm( CsFilRam_OFILM[ UsAryId_OFILM ].UsRamAdd_OFILM, CsFilRam_OFILM[ UsAryId_OFILM ].UlRamDat_OFILM ) ;
			UsAryId_OFILM++ ;
		}
	}
#endif
}



//********************************************************************************
// Function Name 	: IniAdj_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj_OFILM( void )
{
	RegWriteA_Ofilm( WC_RAMACCXY_OFILM, 0x00 ) ;			// 0x018D	Filter copy off

	IniPtAve_OFILM( ) ;								// Average setting
	
	/* OIS */
	RegWriteA_Ofilm( CMSDAC0_OFILM, BIAS_CUR_OIS_OFILM ) ;		// 0x0251	Hall Dac
	RegWriteA_Ofilm( OPGSEL0_OFILM, AMP_GAIN_X_OFILM ) ;			// 0x0253	Hall amp Gain X
	RegWriteA_Ofilm( OPGSEL1_OFILM, AMP_GAIN_Y_OFILM ) ;			// 0x0254	Hall amp Gain Y
	/* AF */
	RegWriteA_Ofilm( CMSDAC1_OFILM, BIAS_CUR_AF_OFILM ) ;			// 0x0252	Hall Dac
	RegWriteA_Ofilm( OPGSEL2_OFILM, AMP_GAIN_AF_OFILM ) ;			// 0x0255	Hall amp Gain AF

	RegWriteA_Ofilm( OSCSET_OFILM, OSC_INI_OFILM ) ;				// 0x0257	OSC ini
	
	/* adjusted value */
	RegWriteA_Ofilm( IZAH_OFILM,	DGYRO_OFST_XH_OFILM ) ;	// 0x02A0		Set Offset High byte
	RegWriteA_Ofilm( IZAL_OFILM,	DGYRO_OFST_XL_OFILM ) ;	// 0x02A1		Set Offset Low byte
	RegWriteA_Ofilm( IZBH_OFILM,	DGYRO_OFST_YH_OFILM ) ;	// 0x02A2		Set Offset High byte
	RegWriteA_Ofilm( IZBL_OFILM,	DGYRO_OFST_YL_OFILM ) ;	// 0x02A3		Set Offset Low byte
	
	/* Ram Access */
	RamAccFixMod_OFILM( ON_OFILM ) ;							// 16bit Fix mode
	
	/* OIS adjusted parameter */
	RamWriteA_Ofilm( DAXHLO_OFILM,		DAHLXO_INI_OFILM ) ;		// 0x1479
	RamWriteA_Ofilm( DAXHLB_OFILM,		DAHLXB_INI_OFILM ) ;		// 0x147A
	RamWriteA_Ofilm( DAYHLO_OFILM,		DAHLYO_INI_OFILM ) ;		// 0x14F9
	RamWriteA_Ofilm( DAYHLB_OFILM,		DAHLYB_INI_OFILM ) ;		// 0x14FA
	RamWriteA_Ofilm( OFF0Z_OFILM,		HXOFF0Z_INI_OFILM ) ;		// 0x1450
	RamWriteA_Ofilm( OFF1Z_OFILM,		HYOFF1Z_INI_OFILM ) ;		// 0x14D0
	RamWriteA_Ofilm( sxg_OFILM,			SXGAIN_INI_OFILM ) ;		// 0x10D3
	RamWriteA_Ofilm( syg_OFILM,			SYGAIN_INI_OFILM ) ;		// 0x11D3
//	UsCntXof = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA_Ofilm( SXOFFZ1,		UsCntXof ) ;		// 0x1461
//	RamWriteA_Ofilm( SYOFFZ1,		UsCntYof ) ;		// 0x14E1

	/* AF adjusted parameter */
	RamWriteA_Ofilm( DAZHLO_OFILM,		DAHLZO_INI_OFILM ) ;		// 0x1529
	RamWriteA_Ofilm( DAZHLB_OFILM,		DAHLZB_INI_OFILM ) ;		// 0x152A

	/* Ram Access */
	RamAccFixMod_OFILM( OFF_OFILM ) ;							// 32bit Float mode
	
	RamWrite32A_Ofilm( gxzoom_OFILM, GXGAIN_INI_OFILM ) ;		// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A_Ofilm( gyzoom_OFILM, GYGAIN_INI_OFILM ) ;		// 0x1120 Gyro Y axis Gain adjusted value

	RamWrite32A_Ofilm( sxq_OFILM, SXQ_INI_OFILM ) ;			// 0x10E5	X axis output direction initial value
	RamWrite32A_Ofilm( syq_OFILM, SYQ_INI_OFILM ) ;			// 0x11E5	Y axis output direction initial value
	
	if( GXHY_GYHX_OFILM ){			/* GX -> HY , GY -> HX */
		RamWrite32A_Ofilm( sxgx_OFILM, 0x00000000 ) ;			// 0x10B8
		RamWrite32A_Ofilm( sxgy_OFILM, 0x3F800000 ) ;			// 0x10B9
		
		RamWrite32A_Ofilm( sygy_OFILM, 0x00000000 ) ;			// 0x11B8
		RamWrite32A_Ofilm( sygx_OFILM, 0x3F800000 ) ;			// 0x11B9
	}
	
	SetZsp_OFILM(0) ;								// Zoom coefficient Initial Setting
	
	RegWriteA_Ofilm( PWMA_OFILM 	, 0xC0 );			// 0x0010		PWM enable

	RegWriteA_Ofilm( STBB0_OFILM 	, 0xDF );			// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA_Ofilm( WC_EQSW_OFILM	, 0x02 ) ;			// 0x01E0
	RegWriteA_Ofilm( WC_MESLOOP1_OFILM	, 0x02 ) ;		// 0x0193
	RegWriteA_Ofilm( WC_MESLOOP0_OFILM	, 0x00 ) ;		// 0x0192
	RegWriteA_Ofilm( WC_AMJLOOP1_OFILM	, 0x02 ) ;		// 0x01A3
	RegWriteA_Ofilm( WC_AMJLOOP0_OFILM	, 0x00 ) ;		// 0x01A2
	
	
	SetPanTiltMode_OFILM( OFF_OFILM ) ;					/* Pan/Tilt OFF_OFILM */

	SetGcf_OFILM( 0 ) ;							/* DI initial value */
#ifdef H1COEF_CHANGER_OFILM
	SetH1cMod_OFILM( ACTMODE_OFILM ) ;					/* Lvl Change Active mode */
#endif
	
	DrvSw_OFILM( ON_OFILM ) ;							/* 0x0001		Driver Mode setting */
	
	RegWriteA_Ofilm( WC_EQON_OFILM, 0x01 ) ;			// 0x0101	Filter ON
}



//********************************************************************************
// Function Name 	: IniCmd_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniCmd_OFILM( void )
{

	MemClr_OFILM( ( unsigned char * )&StAdjPar_OFILM, sizeof( stAdjPar_OFILM ) ) ;	// Adjust Parameter Clear
	
}


//********************************************************************************
// Function Name 	: BsyWit_OFILM
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	BsyWit_OFILM( unsigned short	UsTrgAdr_OFILM, unsigned char	UcTrgDat_OFILM )
{
	unsigned char	UcFlgVal_OFILM ;

	RegWriteA_Ofilm( UsTrgAdr_OFILM, UcTrgDat_OFILM ) ;	// Trigger Register Setting

	UcFlgVal_OFILM	= 1 ;

	while( UcFlgVal_OFILM ) {

		RegReadA_Ofilm( UsTrgAdr_OFILM, &UcFlgVal_OFILM ) ;
		UcFlgVal_OFILM	&= 	( UcTrgDat_OFILM & 0x0F ) ;

		if( CmdRdChk_OFILM() !=0 )	break;		// Dead Lock check (responce check)

	} ;

}


//********************************************************************************
// Function Name 	: MemClr_OFILM
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	MemClr_OFILM( unsigned char	*NcTgtPtr_OFILM, unsigned short	UsClrSiz_OFILM )
{
	unsigned short	UsClrIdx_OFILM ;

	for ( UsClrIdx_OFILM = 0 ; UsClrIdx_OFILM < UsClrSiz_OFILM ; UsClrIdx_OFILM++ )
	{
		*NcTgtPtr_OFILM	= 0 ;
		NcTgtPtr_OFILM++ ;
	}
}



//********************************************************************************
// Function Name 	: WitTim_Ofilm
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
/*void	WitTim_Ofilm( unsigned short	UsWitTim_Ofilm )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim_Ofilm / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}*/

//********************************************************************************
// Function Name 	: GyOutSignal_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal_OFILM( void )
{

	RegWriteA_Ofilm( GRADR0_OFILM,	GYROX_INI_OFILM ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA_Ofilm( GRADR1_OFILM,	GYROY_INI_OFILM ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA_Ofilm( GRSEL_OFILM	, 0x02 );			// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

//********************************************************************************
// Function Name 	: GyOutSignalCont_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Continuosl Function
// History			: First edition 						2013.06.06 Y.Shigeoka
//********************************************************************************
void	GyOutSignalCont_OFILM( void )
{

	/*Start OIS Reading*/
	RegWriteA_Ofilm( GRSEL_OFILM	, 0x04 );			// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#ifdef STANDBY_MODE_OFILM
//********************************************************************************
// Function Name 	: AccWit_OFILM
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	AccWit_OFILM( unsigned char UcTrgDat_OFILM )
{
	unsigned char	UcFlgVal_OFILM ;

	UcFlgVal_OFILM	= 1 ;

	while( UcFlgVal_OFILM ) {
		RegReadA_Ofilm( GRACC_OFILM, &UcFlgVal_OFILM ) ;			// 0x0282
		UcFlgVal_OFILM	&= UcTrgDat_OFILM ;

		if( CmdRdChk_OFILM() !=0 )	break;		// Dead Lock check (responce check)

	} ;

}

//********************************************************************************
// Function Name 	: SelectGySleep_OFILM
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep_OFILM( unsigned char UcSelMode_OFILM )
{
 #ifdef USE_INVENSENSE_OFILM
	unsigned char	UcRamIni_OFILM ;
	unsigned char	UcGrini_OFILM ;

	if(UcSelMode_OFILM == ON_OFILM)
	{
		RegWriteA_Ofilm( WC_EQON_OFILM, 0x00 ) ;		// 0x0101	Equalizer OFF_OFILM
		RegWriteA_Ofilm( GRSEL_OFILM,	0x01 ) ;		/* 0x0280	Set Command Mode			*/

		RegReadA_Ofilm( GRINI_OFILM	, &UcGrini_OFILM );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]
		RegWriteA_Ofilm( GRINI_OFILM, ( UcGrini_OFILM | SLOWMODE_OFILM) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE_OFILM | - | - ]
		
		RegWriteA_Ofilm( GRADR0_OFILM,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_OFILM( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_Ofilm( GRDAT0H_OFILM, &UcRamIni_OFILM ) ;	/* 0x0290 */
		
		UcRamIni_OFILM |= 0x40 ;					/* Set Sleep bit */
  #ifdef GYROSTBY_OFILM
		UcRamIni_OFILM &= ~0x01 ;					/* Clear PLL bit(internal oscillator */
  #endif
		
		RegWriteA_Ofilm( GRADR0_OFILM,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GSETDT_OFILM,	UcRamIni_OFILM ) ;	/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/

  #ifdef GYROSTBY_OFILM
		RegWriteA_Ofilm( GRADR0_OFILM,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GSETDT_OFILM,	0x07 ) ;		/* 0x028A	Set Write Data(STBY ON)	*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
	}
	else
	{
  #ifdef GYROSTBY_OFILM
		RegWriteA_Ofilm( GRADR0_OFILM,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GSETDT_OFILM,	0x00 ) ;		/* 0x028A	Set Write Data(STBY OFF_OFILM)	*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
		RegWriteA_Ofilm( GRADR0_OFILM,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit_OFILM( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA_Ofilm( GRDAT0H_OFILM, &UcRamIni_OFILM ) ;	/* 0x0290 */
		
		UcRamIni_OFILM &= ~0x40 ;					/* Clear Sleep bit */
  #ifdef GYROSTBY_OFILM
		UcRamIni_OFILM |=  0x01 ;					/* Set PLL bit */
  #endif
		
		RegWriteA_Ofilm( GSETDT_OFILM,	UcRamIni_OFILM ) ;	// 0x028A	Set Write Data(Sleep OFF_OFILM)
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		
		RegReadA_Ofilm( GRINI_OFILM	, &UcGrini_OFILM );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE_OFILM | I2CMODE | - ]
		RegWriteA_Ofilm( GRINI_OFILM, ( UcGrini_OFILM & ~SLOWMODE_OFILM) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE_OFILM | I2CMODE | - ]
		
		GyOutSignal_OFILM( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_Ofilm( 50 ) ;						// 50ms wait
		
		RegWriteA_Ofilm( WC_EQON_OFILM, 0x01 ) ;		// 0x0101	GYRO Equalizer ON

		ClrGyr_OFILM( 0x007F , CLR_FRAM1_OFILM );		// Gyro Delay RAM Clear
	}
 #else									/* Panasonic */
	
//	unsigned char	UcRamIni_OFILM ;


	if(UcSelMode_OFILM == ON_OFILM)
	{
		RegWriteA_Ofilm( WC_EQON_OFILM, 0x00 ) ;		// 0x0101	GYRO Equalizer OFF_OFILM
		RegWriteA_Ofilm( GRSEL_OFILM,	0x01 ) ;		/* 0x0280	Set Command Mode			*/
		RegWriteA_Ofilm( GRADR0_OFILM,	0x4C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA_Ofilm( GSETDT_OFILM,	0x02 ) ;		/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/
	}
	else
	{
		RegWriteA_Ofilm( GRADR0_OFILM,	0x4C ) ;		// 0x0283	Set Write Command
		RegWriteA_Ofilm( GSETDT_OFILM,	0x00 ) ;		// 0x028A	Set Write Data(Sleep OFF_OFILM)
		RegWriteA_Ofilm( GRACC_OFILM,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit_OFILM( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		GyOutSignal_OFILM( ) ;					/* Select Gyro output signal 			*/
		
		WitTim_Ofilm( 50 ) ;						// 50ms wait
		
		RegWriteA_Ofilm( WC_EQON_OFILM, 0x01 ) ;		// 0x0101	GYRO Equalizer ON
		ClrGyr_OFILM( 0x007F , CLR_FRAM1_OFILM );		// Gyro Delay RAM Clear
	}
 #endif
}
#endif

#ifdef	GAIN_CONT_OFILM
//********************************************************************************
// Function Name 	: AutoGainControlSw_OFILM
// Retun Value		: NON
// Argment Value	: 0 :OFF_OFILM  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw_OFILM( unsigned char UcModeSw_OFILM )
{

	if( UcModeSw_OFILM == OFF_OFILM )
	{
		RegWriteA_Ofilm( WG_ADJGANGXATO_OFILM, 	0xA0 );					// 0x0129	X exe off
		RegWriteA_Ofilm( WG_ADJGANGYATO_OFILM, 	0xA0 );					// 0x012A	Y exe off
TRACE(" AGC = Off \n") ;
		RamWrite32A_Ofilm( GANADR_OFILM			 , XMAXGAIN_OFILM ) ;			// Gain Through
		RamWrite32A_Ofilm( GANADR_OFILM | 0x0100 , YMAXGAIN_OFILM ) ;			// Gain Through
	}
	else
	{
		RegWriteA_Ofilm( WG_ADJGANGXATO_OFILM, 	0xA3 );					// 0x0129	X exe on
		RegWriteA_Ofilm( WG_ADJGANGYATO_OFILM, 	0xA3 );					// 0x012A	Y exe on
TRACE(" AGC = ON \n") ;
	}

}
#endif


//********************************************************************************
// Function Name 	: ClrGyr_OFILM
// Retun Value		: NON
// Argment Value	: UsClrFil_OFILM - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod_OFILM - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	ClrGyr_OFILM( unsigned short UsClrFil_OFILM , unsigned char UcClrMod_OFILM )
{
	unsigned char	UcRamClr_OFILM;
	unsigned char	count_OFILM = 0; 

	/*Select Filter to clear*/
	RegWriteA_Ofilm( WC_RAMDLYMOD1_OFILM,	(unsigned char)(UsClrFil_OFILM >> 8) ) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA_Ofilm( WC_RAMDLYMOD0_OFILM,	(unsigned char)UsClrFil_OFILM ) ;				// 0x018E		FRAM Initialize Lbyte

	/*Enable Clear*/
	RegWriteA_Ofilm( WC_RAMINITON_OFILM	, UcClrMod_OFILM ) ;	// 0x0102	[ - | - | - | - ][ - | - | Clr ]
	
	/*Check RAM Clear complete*/
//TRACE("  RAM Clear Start.....Fil = %04xh\n", UsClrFil_OFILM );
	do{
		RegReadA_Ofilm( WC_RAMINITON_OFILM, &UcRamClr_OFILM );
		UcRamClr_OFILM &= UcClrMod_OFILM;

		if( count_OFILM++ >= 100 ){
//TRACE("***** ClrGyr_OFILM TimeOut !! *****\n" ) ;
//TRACE_ERROR();
			break;
		}

	}while( UcRamClr_OFILM != 0x00 );
TRACE("  RAM Clear Complete!\n");
}


//********************************************************************************
// Function Name 	: DrvSw_OFILM
// Retun Value		: NON
// Argment Value	: 0:OFF_OFILM  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw_OFILM( unsigned char UcDrvSw_OFILM )
{
	if( UcDrvSw_OFILM == ON_OFILM )
	{
		if( UcPwmMod_OFILM == PWMMOD_CVL_OFILM ) {
			RegWriteA_Ofilm( DRVFC_OFILM	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		} else {
			RegWriteA_Ofilm( DRVFC_OFILM	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
		}
	}
	else
	{
		if( UcPwmMod_OFILM == PWMMOD_CVL_OFILM ) {
			RegWriteA_Ofilm( DRVFC_OFILM	, 0x30 );				// 0x0001	Drvier Block Ena=0
		} else {
			RegWriteA_Ofilm( DRVFC_OFILM	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
		}
	}
}

//********************************************************************************
// Function Name 	: AfDrvSw_OFILM
// Retun Value		: NON
// Argment Value	: 0:OFF_OFILM  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	AfDrvSw_OFILM( unsigned char UcDrvSw_OFILM )
{
	if( UcDrvSw_OFILM == ON_OFILM )
	{
#ifdef	AF_PWMMODE_OFILM
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#else
		RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x20 );				// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
#endif
		RegWriteA_Ofilm( CCAAF_OFILM,   0x80 ) ;				// 0x00A0	[7]=0:OFF_OFILM 1:ON
	}
	else
	{
		RegWriteA_Ofilm( CCAAF_OFILM,   0x00 ) ;				// 0x00A0	[7]=0:OFF_OFILM 1:ON
	}
}

//********************************************************************************
// Function Name 	: RamAccFixMod_OFILM
// Retun Value		: NON
// Argment Value	: 0:OFF_OFILM  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
void	RamAccFixMod_OFILM( unsigned char UcAccMod_OFILM )
{
	switch ( UcAccMod_OFILM ) {
		case OFF_OFILM :
			RegWriteA_Ofilm( WC_RAMACCMOD_OFILM,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
			break ;
		case ON_OFILM :
			RegWriteA_Ofilm( WC_RAMACCMOD_OFILM,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
			break ;
	}
}
	

//********************************************************************************
// Function Name 	: IniAf_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniAf_OFILM( void )
{
	unsigned char	UcStbb0_OFILM ;
	
	AfDrvSw_OFILM( OFF_OFILM ) ;								/* AF Drvier Block Ena=0 */
#ifdef	AF_PWMMODE_OFILM
	RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x00 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
#else
	RegWriteA_Ofilm( DRVFCAF_OFILM	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
#endif
	RegWriteA_Ofilm( DRVFC3AF_OFILM	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
	RegWriteA_Ofilm( DRVFC4AF_OFILM	, 0x80 );					// 0x0084	DOFSTDAF
	RegWriteA_Ofilm( PWMAAF_OFILM,    0x00 ) ;					// 0x0090	AF PWM standby
	RegWriteA_Ofilm( AFFC_OFILM,   0x80 ) ;						// 0x0088	OpenAF/-/-
#ifdef	AF_PWMMODE_OFILM
	RegWriteA_Ofilm( DRVFC2AF_OFILM,    0x82 ) ;				// 0x0082	AF slope3
	RegWriteA_Ofilm( DRVCH3SEL_OFILM,   0x02 ) ;				// 0x0085	AF only IN1 control
	RegWriteA_Ofilm( PWMFCAF_OFILM,     0x89 ) ;				// 0x0091	AF GND , Carrier , MODE1 
	RegWriteA_Ofilm( PWMPERIODAF_OFILM, 0xA0 ) ;				// 0x0099	AF none-synchronism
#else
	RegWriteA_Ofilm( DRVFC2AF_OFILM,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA_Ofilm( DRVCH3SEL_OFILM,   0x00 ) ;				// 0x0085	AF H bridge control
	RegWriteA_Ofilm( PWMFCAF_OFILM,     0x01 ) ;				// 0x0091	AF VREF , Carrier , MODE1
	RegWriteA_Ofilm( PWMPERIODAF_OFILM, 0x20 ) ;				// 0x0099	AF none-synchronism
#endif
	RegWriteA_Ofilm( CCFCAF_OFILM,   0x40 ) ;					// 0x00A1	GND/-
	
	RegReadA_Ofilm( STBB0_OFILM 	, &UcStbb0_OFILM );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0_OFILM &= 0x7F ;
	RegWriteA_Ofilm( STBB0_OFILM, UcStbb0_OFILM ) ;			// 0x0250	OIS standby
	RegWriteA_Ofilm( STBB1_OFILM, 0x00 ) ;				// 0x0264	All standby
	
	/* AF Initial setting */
	RegWriteA_Ofilm( FSTMODE_OFILM,		FSTMODE_AF_OFILM ) ;		// 0x0302
	RamWriteA_Ofilm( RWEXD1_L_OFILM,	RWEXD1_L_AF_OFILM ) ;		// 0x0396 - 0x0397 (Register continuos write)
	RamWriteA_Ofilm( RWEXD2_L_OFILM,	RWEXD2_L_AF_OFILM ) ;		// 0x0398 - 0x0399 (Register continuos write)
	RamWriteA_Ofilm( RWEXD3_L_OFILM,	RWEXD3_L_AF_OFILM ) ;		// 0x039A - 0x039B (Register continuos write)
	RegWriteA_Ofilm( FSTCTIME_OFILM,	FSTCTIME_AF_OFILM ) ;		// 0x0303 	
	RamWriteA_Ofilm( TCODEH_OFILM,		0x0000 ) ;			// 0x0304 - 0x0305 (Register continuos write)
	
#ifdef	AF_PWMMODE_OFILM
	RegWriteA_Ofilm( PWMAAF_OFILM,    0x80 ) ;			// 0x0090	AF PWM enable
#endif

	UcStbb0_OFILM |= 0x80 ;
	RegWriteA_Ofilm( STBB0_OFILM, UcStbb0_OFILM ) ;			// 0x0250	
	RegWriteA_Ofilm( STBB1_OFILM	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	AfDrvSw_OFILM( ON_OFILM ) ;								/* AF Drvier Block Ena=1 */
}



//********************************************************************************
// Function Name 	: IniPtAve_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtAve_OFILM( void )
{
	RegWriteA_Ofilm( WG_PANSTT1DWNSMP0_OFILM, 0x00 );		// 0x0134
	RegWriteA_Ofilm( WG_PANSTT1DWNSMP1_OFILM, 0x00 );		// 0x0135
	RegWriteA_Ofilm( WG_PANSTT2DWNSMP0_OFILM, 0x90 );		// 0x0136 400
	RegWriteA_Ofilm( WG_PANSTT2DWNSMP1_OFILM, 0x01 );		// 0x0137
	RegWriteA_Ofilm( WG_PANSTT3DWNSMP0_OFILM, 0x64 );		// 0x0138 100
	RegWriteA_Ofilm( WG_PANSTT3DWNSMP1_OFILM, 0x00 );		// 0x0139
	RegWriteA_Ofilm( WG_PANSTT4DWNSMP0_OFILM, 0x00 );		// 0x013A
	RegWriteA_Ofilm( WG_PANSTT4DWNSMP1_OFILM, 0x00 );		// 0x013B

	RamWrite32A_Ofilm( st1mean_OFILM, 0x3f800000 );		// 0x1235
	RamWrite32A_Ofilm( st2mean_OFILM, 0x3B23D700 );		// 0x1236	1/400
	RamWrite32A_Ofilm( st3mean_OFILM, 0x3C23D700 );		// 0x1237	1/100
	RamWrite32A_Ofilm( st4mean_OFILM, 0x3f800000 );		// 0x1238
			
}
	
//********************************************************************************
// Function Name 	: IniPtMovMod_OFILM
// Retun Value		: NON
// Argment Value	: OFF_OFILM:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtMovMod_OFILM( unsigned char UcPtMod_OFILM )
{
	switch ( UcPtMod_OFILM ) {
		case OFF_OFILM :
			RegWriteA_Ofilm( WG_PANSTTSETGYRO_OFILM, 	0x00 );		// 0x0154
			RegWriteA_Ofilm( WG_PANSTTSETGAIN_OFILM, 	0x54 );		// 0x0155
			RegWriteA_Ofilm( WG_PANSTTSETISTP_OFILM, 	0x14 );		// 0x0156
			RegWriteA_Ofilm( WG_PANSTTSETIFTR_OFILM,	0x94 );		// 0x0157
			RegWriteA_Ofilm( WG_PANSTTSETLFTR_OFILM,	0x00 );		// 0x0158

			break ;
		case ON_OFILM :
			RegWriteA_Ofilm( WG_PANSTTSETGYRO_OFILM, 	0x00 );		// 0x0154
			RegWriteA_Ofilm( WG_PANSTTSETGAIN_OFILM, 	0x00 );		// 0x0155
			RegWriteA_Ofilm( WG_PANSTTSETISTP_OFILM, 	0x14 );		// 0x0156
			RegWriteA_Ofilm( WG_PANSTTSETIFTR_OFILM,	0x94 );		// 0x0157
			RegWriteA_Ofilm( WG_PANSTTSETLFTR_OFILM,	0x00 );		// 0x0158
			break ;
	}
}

//********************************************************************************
// Function Name 	: ChkCvr_OFILM
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2013.10.03 Y.Shigeoka
//********************************************************************************
void	ChkCvr_OFILM( void )
{
	RegReadA_Ofilm( CVER_OFILM ,	&UcCvrCod_OFILM );		// 0x027E
	RegWriteA_Ofilm( MDLREG_OFILM ,	MDL_VER_OFILM );			// 0x00FF	Model
	RegWriteA_Ofilm( VRREG_OFILM ,	FW_VER_OFILM );			// 0x02D0	Version
}

