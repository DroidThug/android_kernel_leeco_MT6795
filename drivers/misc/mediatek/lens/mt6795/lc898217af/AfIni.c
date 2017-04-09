//********************************************************************************
//
//		<< LC898217 Evaluation Soft >>
//		Program Name	: AfIni.c
//		Design			: Y.Yamada
//		History			: First edition						2015.07.03 Y.Tashita
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include	"Af.h"
#include <linux/xlog.h>

#define AF_DRVNAME "LC898217AF"

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)	pr_debug(AF_DRVNAME "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif

//********************************************************************************
// Function Name 	: CheckCver
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Check CVER register Function
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char CheckCver( void )
{
	unsigned char UcLsiVer;

	RegReadA_8( CVER, &UcLsiVer );
	LOG_INF("[CheckCver]CVERB3 = %x \n",UcLsiVer);
	return( UcLsiVer == 0x71 ) ? SUCCESS : FAILURE ;
}

//********************************************************************************
// Function Name 	: WakeUpCheck
// Retun Value		: unsigned char UcWakeUpSts
// Argment Value	: Void
// Explanation		: WakeUp Check for autodownload
// History		: First edition 		     2015.07.11 John.Jeong
//********************************************************************************
unsigned char	WakeUpCheck(unsigned char UcRescailMode)
{
	unsigned char	UcStatus, UcReadDat ;
	unsigned short	i ;
	
	//I2C communication check if necessary	
	UcStatus	= CheckCver() ;

	if( UcStatus != FAILURE ) {
		for( i = 0 ; i < 3000 ; i++  ) {
			RegReadA_8( FUNCRSLT1, &UcReadDat ) ;
	LOG_INF("[WakeUpCheck]FUNCRSLT1 B3 = %x \n",UcReadDat);
			if(UcRescailMode == RESCAILING){
				if( UcReadDat == 0x00 )             //Rescailing Mode
					break ;
			}
			else
			{
				if( !UcReadDat )
					break ;			
			}
		}

		if( i == 3000 ) {
	LOG_INF("[WakeUpCheck]i == 3000 DOWNLOAD_ERROR! \n");
			UcStatus	= DOWNLOAD_ERROR ;
		}
	}
		
	//RegWriteA_8( PINC, 0x02 ) ;
	//RegWriteA_8( TESTC, 0x20 ) ;

	
	return( UcStatus ) ;
}

//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
/*void	WitTim( unsigned short	UsWitTim )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}
*/

//********************************************************************************
// Function Name 	: RescailEn
// Retun Value		: Void
// Argment Value	: Void
// Explanation		: Rescail Enable
// History		: First edition 		    2015.07.11 John.Jeong
//******************************************************************************** //FUNCRUN2
void RescailEn( void )
{
	RegWriteA_8(FUNCRUN2	, 0x02 );  //0xA1 02h, After Calibration Data Writing, Rescailing Run 
	WitTim(1) ;//WAIT	1ms
}

//********************************************************************************
// Function Name 	: Stmv217
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter, Target Position
// Explanation		: Stpmove Function
// History		: First edition 		    2015.07.08 John.Jeong
//********************************************************************************
#define TIME_OUT 100
#define TIMEOUT_ERROR 10

unsigned char Stmv217( unsigned char DH, unsigned char DL )
{
	unsigned char	UcConvCheck;
	unsigned char	UcSetTime;
	unsigned char	UcCount;	
	
	//AF Operation
	RegWriteA_8(TARGETH	, DH );  //0x84, DH 0x0X
	RegWriteA_8(TARGETL	, DL );  //0x85, DL 0xXX
	WitTim(2) ;//WAIT	5ms
	UcCount = 0;
	do
	{
		RegReadA_8(SRVSTATE1, &UcConvCheck); 	//0xB0
		UcConvCheck = UcConvCheck & 0x80;  	//bit7 == 0 or not
		RegReadA_8(TGTCNVTIM, &UcSetTime); 	//0xB2 Settle Time check Settle Time Calculate =
							// ( B2h Read Value x 0.171mS ) - 2.731mS Condition:
							//EEPROM 55h(THD1) = 0x20
							//82h:bit2-0( LOCCNVCNT ) = 100b
		WitTim(1) ;//WAIT 2ms
		UcCount++;
		LOG_INF("[Stmv217]Stmv217 UcCount = %d \n",UcCount);
		
		if(UcCount == TIME_OUT)	{
			return TIMEOUT_ERROR;
		}
	}while(UcConvCheck); 	
	
	return SUCCESS;
}


//********************************************************************************
// Function Name 	: ReadSRAM
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: ReadSRAM
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char ReadSRAM( void )
{
	unsigned char UcSRAMdata;
	int i;
	 //000h ~ 04Fh
	for( i = 0 ; i <= 0x4F ; i++ )
	{
		RegReadA_8( i, &UcSRAMdata );
		LOG_INF("[ReadSRAM]UcSRAMdata[%x] = %x \n",i, UcSRAMdata);
	}

	return UcSRAMdata;
}


//********************************************************************************
// Function Name 	: ReadEEPROM
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: ReadEEPROM
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char ReadEEPROM( void )
{
	unsigned char UcEEPROMdata;
	unsigned char UcEnDATA;
	unsigned short Uc16EEPROMdata;

	int i;
	 //000h ~ 04Fh
			WitTim(100);

	RegReadA_8( 0x80, &UcEnDATA );
	LOG_INF("[ReadSRAM]UcEnDATA_1 = %x \n", UcEnDATA);
	UcEnDATA = (UcEnDATA | 0x01);
	LOG_INF("[ReadSRAM]UcEnDATA_2 = %x \n", UcEnDATA);
	
 	RegWriteA_8(0x80, UcEnDATA );  //0x80, DH 0x0X
 	
	for( i = 0 ; i <= 0x7F ; i++ )
	{
		//RegReadA_8_EEPROM( i, &UcEEPROMdata );
		RamReadA_8_EEPROM( i, &Uc16EEPROMdata );
		LOG_INF("[ReadSRAM]Uc16EEPROMdata[%x] = %x \n",i, Uc16EEPROMdata);
		i++;
	}
	
	RamReadA_8_EEPROM( 0x14, &Uc16EEPROMdata );
	LOG_INF("[WriteEEPROM]EEPROM14 = %x \n", Uc16EEPROMdata);

	//RegReadA_8_EEPROM( 0x14, &UcEEPROMdata );
	//LOG_INF("[ReadEEPROM]EEPROM14 = %x \n", UcEEPROMdata);
	//RegReadA_8_EEPROM( 0x15, &UcEEPROMdata );
	//LOG_INF("[ReadEEPROM]EEPROM15 = %x \n", UcEEPROMdata);
	
	RegReadA_8( 0x80, &UcEnDATA );
	LOG_INF("[ReadSRAM]UcEnDATA_3 = %x \n", UcEnDATA);
	UcEnDATA = (UcEnDATA & 0xFE);
	LOG_INF("[ReadSRAM]UcEnDATA_4 = %x \n", UcEnDATA);
	
 	RegWriteA_8(0x80, UcEnDATA );  //0x80, DH 0x0X
			WitTim(100);

	return UcEEPROMdata;
}


//********************************************************************************
// Function Name 	: WriteEEPROM
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: WriteEEPROM
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char WriteEEPROM(void)
{
        unsigned char UcEEPROMdata;
        unsigned char UcEnDATA;
        unsigned short Uc16EEPROMdata;
        int i;
        static int first_enter = 0;

        WitTim(100);

        //000h ~ 04Fh
	    if (first_enter == 0)
        {
		        WitTim(10000);
        
                RegReadA_8( 0x80, &UcEnDATA );
                LOG_INF("[ReadSRAM]UcEnDATA_1 = %x \n", UcEnDATA);
                UcEnDATA = (UcEnDATA | 0x01);
                LOG_INF("[ReadSRAM]UcEnDATA_2 = %x \n", UcEnDATA);

                
               RegWriteA_8(0x80, UcEnDATA );  //0x80, DH 0x0X
                
               RegReadA_8_EEPROM( 0x09, &UcEEPROMdata );
               LOG_INF("[WriteEEPROM]EEPROM109 = %x \n", UcEEPROMdata);

               RegWriteA_8(0x99, 0xAF );  //0x80, DH 0x0X
               RegWriteA_8(0x9A, 0x85 );  //0x80, DH 0x0X
               

               RegWriteA_8_EEPROM(0x09, 0xC0);
               RegReadA_8_EEPROM( 0x09, &UcEEPROMdata );
               LOG_INF("[WriteEEPROM]EEPROM109 = %x \n", UcEEPROMdata);
				
               RegWriteA_8(0x99, 0x00 );  //0x80, DH 0x0X
               RegWriteA_8(0x9A, 0x00 );  //0x80, DH 0x0X
               
                RegReadA_8( 0x80, &UcEnDATA );
                LOG_INF("[ReadSRAM]UcEnDATA_3 = %x \n", UcEnDATA);
                UcEnDATA = (UcEnDATA & 0xFE);
                LOG_INF("[ReadSRAM]UcEnDATA_4 = %x \n", UcEnDATA);
                
               RegWriteA_8(0x80, UcEnDATA );  //0x80, DH 0x0X
		        WitTim(10000);

                first_enter = 1;
        }
        return UcEEPROMdata;
}


//********************************************************************************
// Function Name 	: SetPosition
// Retun Value		: NON
// Argment Value	: unsigned char UcPosition
// Explanation		: AF Operation function Range 1023 to 0
// History		: First edition 		2015.07.13 John Jeong
//********************************************************************************
unsigned char SetPosition_217(unsigned short UsPosition)
{
    unsigned char UcPosH;
    unsigned char UcPosL;
	unsigned char UcAFSts;
	unsigned char UcTemp;

    //unsigned char UcMaxPosition = 1023;

	UsPosition = 1023 - UsPosition;

    UcPosH = (unsigned char)(UsPosition >> 8);
	UcPosL = (unsigned char)(UsPosition & 0x00FF);    
   
	LOG_INF("[SetPosition_217]UcPosH = %x UcPosL = %x \n", UcPosH,UcPosL);
    UcAFSts = Stmv217( UcPosH, UcPosL );   

	//RegReadA_8( 0x84, &UcTemp );
	//LOG_INF("[SetPosition_217]0x84 = %x \n", UcTemp);
	//RegReadA_8( 0x85, &UcTemp );
	//LOG_INF("[SetPosition_217]0x85 = %x \n", UcTemp);

	//RegReadA_8( 0x0A, &UcTemp );
	//LOG_INF("[SetPosition_217]0x0A = %x \n", UcTemp);
	//RegReadA_8( 0x0B, &UcTemp );
	//LOG_INF("[SetPosition_217]0x0B = %x \n", UcTemp);
	
    if (UcAFSts != SUCCESS)
	return FAILURE;	

    return  UcAFSts; 
}


#define AUTO_DOWNLOAD 1

extern unsigned short read_3l9_afinfi;
extern unsigned short read_3l9_afmacro;

//********************************************************************************
// Function Name 	: Init
// Retun Value		: unsigned char
// Argment Value	: NON
// Explanation		: Calibration and Initialize
// History		: First edition 		2015.07.13 John Jeong
//********************************************************************************
unsigned char Init_217_auto( void )
{
	unsigned char	UcIniSts ;	
	unsigned char	UcAFSts ;
	unsigned char	UcSts ;
	unsigned char	UcDownloadMode = AUTO_DOWNLOAD;	
	unsigned char	UcMode = RESCAILING ;
	unsigned char	UcLensPosition;
    unsigned char UcPosH;
    unsigned char UcPosL;
	unsigned short af_infinit;	//af_infinit 0x0011
	unsigned short af_macro;	//af_macro 0x0011
	unsigned short af_init;	//af_init 0x0011
	
	//After Power On
	WitTim(100); //Wait 8ms, Make sure VDD Stabile,
		   //This is one example of time. Please adjust wait time to fit your system.
		   //And change WitTim() to your system delay function.
	//ReadSRAM();
    //ReadEEPROM();
    //WriteEEPROM();
		   
	if( UcDownloadMode == AUTO_DOWNLOAD){
		UcSts = WakeUpCheck(RESCAILING);
		if (UcSts != SUCCESS)
			return FAILURE;	

	}
	/*
	else( UcDownloadMode == MANUAL_DOWNLOAD){
		UcIniSts = IniSet(MANUAL_DOWNLOAD); 
		if (UcIniSts != SUCCESS)
			return FAILURE;	
	}
	*/

	//if(UcMode ==  RESCAILING){
		//RescailEn(); //A1h 02h setting
	//}

	//Set Init Position
	af_infinit = read_3l9_afinfi;
	af_macro = read_3l9_afmacro;
	
	if(af_infinit > 1023)
	{
		af_infinit = 1023;
	}

	if(af_infinit < 0)
	{
		af_infinit = 0;
	}

	if(af_macro > 1023)
	{
		af_macro = 1023;
	}

	if(af_macro < 0)
	{
		af_macro = 0;
	}
	
	LOG_INF("[Init_217]g_s5k3l9_otp_struct.af_infinit = %x \n", af_infinit);
	LOG_INF("[Init_217]g_s5k3l9_otp_struct.af_macro = %x \n", af_macro);
	
	af_init = 1023 - ((af_infinit+af_macro)/2);
	
	UcPosH = (unsigned char)((af_init) >> 8);
	UcPosL = (unsigned char)((af_init) & 0x00FF);    
	
	LOG_INF("[Init_217]af_infinit = %x \n", af_infinit);
	
	//UcAFSts = Stmv217( 0x01, 0x00 );   //Inintial position ex. 0x0100
	UcAFSts = Stmv217( UcPosH, UcPosL );   //Inintial position ex. 0x0100

	//RegWriteA_8( 0xA0, 0x81); //for smooth moving.
	
	if (UcAFSts != SUCCESS)
		return FAILURE;	
		
	//Lens Posintion Reading need to bit shift >> 4
	//RegReadA( PIDI, &UcLensPosition); // Lens Position Read
	//UcLensPosition = UcLensPosition >> 4;

	return SUCCESS;	
}
//********************************************************************************
// Function Name 	: IniSet
// Retun Value		: NON
// Argment Value	: Hall Offset Bias
// Explanation		: Initial Setting Function
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char	IniSet( unsigned char UcAutoDownload,unsigned char UcRescailMode)
{
	unsigned char	UcStatus, UcReadDat ;
	unsigned short	i ;
	
	//UcStatus	= CheckCver() ;

	if( UcStatus != FAILURE ) {
		//IniCmd() ;
		
		if( UcAutoDownload != WITHOUT_DOWNLOAD ) {
			RegWriteA_8( ADWLCTL, 0x01 ) ;
			
			for( i = 0 ; i < 3000 ; i++  ) {
				RegReadA_8( FUNCRSLT1, &UcReadDat ) ;
				LOG_INF("[IniSet]UcReadDat = %x \n", UcReadDat);

				if(UcRescailMode == RESCAILING){
					if( UcReadDat == 0x01 )             //Rescailing Mode
					{
						LOG_INF("[IniSet]RESCAILING i = %x \n", i);
						break ;						
					}
				}
				else
				{
					if( !UcReadDat )
					{
						LOG_INF("[IniSet]NOT RESCAILING i = %x \n", i);
						break ;						
					}
				}
			}
			
			if( i == 3000 ) {
				UcStatus	= DOWNLOAD_ERROR ;
				LOG_INF("[IniSet]DOWNLOAD_ERROR !! \n");
			}
		}
		
		RegWriteA_8( PINC, 0x02 ) ;
		RegWriteA_8( TESTC, 0x20 ) ;
	}

	return( UcStatus ) ;
}



//********************************************************************************
// Function Name 	: Init
// Retun Value		: unsigned char
// Argment Value	: NON
// Explanation		: Calibration and Initialize
// History		: First edition 		2015.07.13 John Jeong
//********************************************************************************
unsigned char Init_217_manual( void )
{
	unsigned char	UcSts ;
	//unsigned char	UcDownloadMode = AUTO_DOWNLOAD;	
	//unsigned char	UcMode = RESCAILING ;
	
	//After Power On
	WitTim(8); //Wait 8ms, Make sure VDD Stabile,
		   //This is one example of time. Please adjust wait time to fit your system.
		   //And change WitTim() to your system delay function.
    //ReadEEPROM();
    //WriteEEPROM();
		   
	//UcSts = WakeUpCheck(RESCAILING);
	UcSts = IniSet(MANUAL_DOWNLOAD,RESCAILING);	
		if (UcSts != SUCCESS)
			return FAILURE;	

	RescailEn(); //A1h 02h setting

	SetPosition_217(0x100);
		
	return SUCCESS;	
}
unsigned char Init_217( void )
{
	//Init_217_manual();
	Init_217_auto();
	
}


