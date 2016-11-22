//********************************************************************************
//
//		<< LC898217 Evaluation Soft>>
//		Program Name	: Af.h
// 		Explanation		: LC898217 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition						2015.07.03 Y.Tashita
//********************************************************************************


#define	FW_VER			0x0003

#ifdef	AFINI
	#define	AFINI__
#else
	#define	AFINI__		extern
#endif

#ifdef	AFCMD
	#define	AFCMD__
#else
	#define	AFCMD__		extern
#endif

// Common Define
#define	SUCCESS		0x00		// Success
#define	FAILURE		0x01		// Failure
#define RESCAILING 	1
#define	DOWNLOAD_ERROR	2
#define	NOP_TIME		0.00004166F

#define	WITHOUT_DOWNLOAD	0
#define	AUTO_DOWNLOAD		1
#define	MANUAL_DOWNLOAD		2

/*******************************************************************************
 *  AfDef.h - Header file for LC898217
 *
 *  ON Semiconductor
 *
 *  REVISION:
 *      2015/07/03 - First Edition, Y.Tashita
 ******************************************************************************/
// Delay RAM    00h ~ 4Fh
#define	ADHI		0x00
#define	ADHIL		0x01
#define	PIDO		0x02
#define	PIDOL		0x03
#define	TREG		0x04
#define	TREGL		0x05
#define	COFF		0x06
#define	COFFL		0x07
#define	DOFF		0x08
#define	DOFFL		0x09
#define	PIDI		0x0A
#define	PIDIL		0x0B
#define	EZ			0x0C
#define	EZL			0x0D
#define	DZ1			0x0E
#define	DZ1L		0x0F
#define	DZ2			0x10
#define	DZ2L		0x11
#define	IZ1			0x12
#define	IZ1L		0x13
#define	IZ2			0x14
#define	IZ2L		0x15
#define	OZ1			0x16
#define	OZ1L		0x17
#define	OZ2			0x18
#define	OZ2L		0x19
#define	gain1		0x1A
#define	gain1L		0x1B
#define	sing		0x1C
#define	singL		0x1D
#define	CTMP		0x1E
#define	CTMPL		0x1F
#define	MS1Z11		0x20
#define	MS1Z11L		0x21
#define	MS1Z12		0x22
#define	MS1Z12L		0x23
#define	MS1Z22		0x24
#define	MS1Z22L		0x25
#define	MS2Z11		0x26
#define	MS2Z11L		0x27
#define	MS2Z12		0x28
#define	MS2Z12L		0x29
#define	MS2Z12		0x28
#define	MS2Z12L		0x29
#define	MS2Z22		0x2A
#define	MS2Z22L		0x2B
#define	MSR1CH		0x2C
#define	MSR1CHL		0x2D
#define	MSR1CL		0x2E
#define	MSR1CLL		0x2F
#define	MSR2CH		0x30
#define	MSR2CHL		0x31
#define	MSR2CL		0x32
#define	MSR2CLL		0x33
#define	PeakMax		0x34
#define	PeakMin		0x35
#define	TgtMax		0x36
#define	TgtMin		0x37
#define	Margin		0x38
#define	MarginL		0x39
#define	AjCoef		0x3A
#define	AjCoefL		0x3B
//#define				0x3C
//#define				0x3D
#define	DAHLO		0x3E
#define	DAHLB		0x3F
#define	TIN		0x40
#define	TINL		0x41
#define	TTMP		0x42
#define	TTMPL		0x43
#define	RsCoef		0x44
#define	RsCoefL		0x45
#define	RsCoefIv	0x46
#define	RsCoefIvL	0x47
#define	RsTmp		0x48
#define	RsTmpL		0x49
#define	TOFF		0x4A
#define	TOFFL		0x4B
#define	CountThd	0x4C
#define	CountThdL	0x4D
#define	StackBt		0x4E
#define	StackBtL	0x4F

//Coefficient RAM 140h ~ 17Fh
#define	ag		0x140
#define	agLpgH		0x141
#define	pg		0x142
#define	ig		0x143
#define	igLifcH		0x144
#define	ifc		0x145
#define	LIMIT1		0x146
#define	LIMIT2		0x147
#define	da		0x148
#define	daLdbH		0x149
#define	db		0x14A
#define	dc		0x14B
#define	dcLdgH		0x14C
#define	dg		0x14D
#define	SFTX1		0x14E
#define	SFTX2		0x14F
#define	brkg		0x150
#define	brkgLkickgH	0x151
#define	kickg		0x152
#define	ofc		0x153
#define	ofcL		0x154
#define	THD1		0x155
#define	THD2		0x156
#define	THD3		0x157
#define	BRKFIXV		0x158
#define	KICKFIXV	0x159
#define	THD4		0x15A
#define	THD5		0x15B
#define	THD6		0x15C
#define	THD7		0x15D
#define	THD8		0x15E
#define	THD9		0x15F
#define	RevMax		0x160
#define	RevMaxLRevMinH	0x161
#define	RevMin		0x162
#define	CoilVal		0x163
#define	CoilValL	0x164
#define	TempSlopeV	0x165
#define	TempSlopeVL	0x166
#define	HallMax		0x167
#define	HallMaxLMinH	0x168
#define	HallMin		0x169
#define	LinearMax	0x16A
#define	LinearMaxLMinH	0x16B
#define	LinearMin	0x16C
#define	Macro		0x16D
#define	MacroLInfiniH	0x16E
#define	Infini		0x16F
#define	Linear1		0x170
#define	Linear1L2H	0x171
#define	Linear2		0x172
#define	Linear3		0x173
#define	Linear3L4H	0x174
#define	Linear4		0x175
#define	Linear5		0x176
#define	Linear5L6H	0x177
#define	Linear6		0x178
#define	Linear7		0x179
#define	Linear7L8H	0x17A
#define	Linear8		0x17B
//#define				0x17C
#define	CheckCode1	0x17D
#define	CheckCode2	0x17E
#define	CheckCode3	0x17F

//Register 80h ~ F9h
#define	INPUT		0x80
#define	ANLG		0x81
#define	PID1		0x82
#define	PID2		0x83
#define	TARGETH		0x84
#define	TARGETL		0x85
#define	STMVH		0x86
#define	STMVL		0x87
#define	FSTH		0x88
#define	FSTL		0x89
#define	SWFC		0x8A
#define	SWEN		0x8B
#define	MSSET		0x8C
#define	MSNUM		0x8D
#define	CCFC		0x8E
#define	ASW		0x8F
#define	IDSEL		0x90
#define	OSCS		0x91
#define	DRVGAIN		0x92
#define	DRVOFST1	0x93
#define	DRVOFST2	0x94
#define	LDOSFT		0x95
#define	CLKSEL		0x96
#define	STBY		0x97
#define	SOFTRES		0x98
#define	E2WPMS1		0x99
#define	E2WPMS2		0x9A
#define	TWCNT		0x9B
//#define				0x9C
//#define				0x9D
//#define				0x9E
//#define				0x9F
#define	FUNCRUN1	0xA0
#define	FUNCRUN2	0xA1
#define	FUNCOPT1	0xA2
#define	FUNCE2PW	0xA3
//#define				0xA4
//#define				0xA5
//#define				0xA6
//#define				0xA7
//#define				0xA8
//#define 				0xA9
//#define				0xAA
//#define				0xAB
//#define				0xAC
//#define				0xAD
//#define				0xAE
//#define				0xAF
#define	SRVSTATE1	0xB0
#define	SRVSTATE2	0xB1
#define	TGTCNVTIM	0xB2
#define	FUNCRSLT1	0xB3
#define	FUNCRSLT2	0xB4
#define	FOPECNT		0xB5
#define	FSTATE		0xB6
#define	FCNT0		0xB7
#define	FCNT1		0xB8
//#define				0xB9
//#define				0xBA
//#define				0xBB
//#define				0xBC
//#define				0xBD
//#define				0xBE
//#define				0xBF
#define	DLYMON		0xC0
#define	DLYMON2		0xC1
#define	DLYDAT1H	0xC2
#define	DLYDAT1L	0xC3
#define	DLYDAT2H	0xC4
#define	DLYDAT2L	0xC5
#define	MON		0xC6
//#define				0xC7
//#define				0xC8
//#define				0xC9
//#define				0xCA
//#define				0xCB
//#define				0xCC
//#define				0xCD
//#define				0xCE
//#define				0xCF
//#define				0xD0
//#define				0xD1
//#define				0xD2
//#define				0xD3
//#define				0xD4
//#define				0xD5
//#define				0xD6
//#define				0xD7
//#define				0xD8
//#define				0xD9
//#define				0xDA
//#define				0xDB
//#define				0xDC
//#define				0xDD
//#define				0xDE
//#define				0xDF
#define	ADWLCTL		0xE0
#define	E2PBUSY		0xE1
#define	EEPSMOD		0xE2
#define	EEPTEST		0xE3
//#define				0xE4
//#define				0xE5
//#define				0xE6
//#define				0xE7
//#define				0xE8
//#define				0xE9
//#define				0xEA
//#define				0xEB
//#define				0xEC
//#define				0xED
//#define				0xEE
//#define				0xEF
#define CVER			0xF0
#define A2BMODE			0xF1
#define TESTC			0xF2
#define PINC			0xF3
#define DRVT1			0xF4
#define DRVT2			0xF5
#define STBY2			0xF6
#define STBY3			0xF7
#define TESTAL			0xF8
#define TEST			0xF9


// Prottype Declation
AFINI__	unsigned char 		CheckCver( void );
AFINI__ unsigned char		WakeUpCheck(unsigned char ) ;											// Initial Top Function
//AFINI__ void			WitTim( unsigned short ) ;											// Wait
AFINI__ unsigned char 		Init( void );
AFINI__ void 			RescailEn( void );
AFINI__ unsigned char 		SetPosition(unsigned short UsPosition);


/*
void RegWriteA(unsigned short RegAddr, unsigned char RegData);
void RegReadA(unsigned short RegAddr, unsigned char *RegData);
void RamWriteA( unsigned short RamAddr, unsigned short RamData );
void RamReadA( unsigned short RamAddr, void * ReadData );
void RamWrite32A(unsigned short RamAddr, unsigned long RamData );
void RamRead32A(unsigned short RamAddr, void * ReadData );
*/
void RegWriteA_8(unsigned char RegAddr, unsigned char RegData);
void RegReadA_8(unsigned char RegAddr, unsigned char *RegData);
void RamWriteA_8( unsigned char RamAddr, unsigned short RamData );
void RamReadA_8( unsigned char RamAddr, void * ReadData );
void RamWrite32A_8(unsigned char RamAddr, unsigned long RamData );
void RamRead32A_8(unsigned char RamAddr, void * ReadData );
void RegReadA_8_EEPROM(unsigned char RegAddr, unsigned char *RegData);
void RegWriteA_8_EEPROM(unsigned char RegAddr, unsigned char RegData);
void RamReadA_8_EEPROM( unsigned char RamAddr, void * ReadData );
void RamWriteA_8_EEPROM( unsigned char RamAddr, unsigned short RamData );

void WitTim(unsigned short  UsWitTim );


