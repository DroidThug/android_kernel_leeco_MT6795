/*******************************************************************************
 *  OisDef.h - Header file for LC898122
 *
 *  ON Semiconductor
 *
 *  REVISION:
 *      2013/01/07 - First Edition, Y.Shigeoka
 ******************************************************************************/

//==============================================================================
//PWM Register
//==============================================================================
//#define							0x0000
#define		DRVFC_OFILM				0x0001
#define		DRVFC2_OFILM				0x0002
#define		DRVSELX_OFILM				0x0003
#define		DRVSELY_OFILM				0x0004
#define		DRVCH1SEL_OFILM			0x0005
#define		DRVCH2SEL_OFILM			0x0006
//#define							0x0007
//#define							0x0008
//#define							0x0009
//#define							0x000A
//#define							0x000B
//#define							0x000C
//#define							0x000D
//#define							0x000E
//#define							0x000F
#define		PWMA_OFILM				0x0010
#define		PWMFC_OFILM				0x0011
#define		PWMDLYX_OFILM				0x0012
#define		PWMDLYY_OFILM				0x0013
#define		PWMDLYTIMX_OFILM			0x0014
#define		PWMDLYTIMY_OFILM			0x0015
#define		PWMFC2_OFILM				0x0016
//#define							0x0017
#define		PWMPERIODX_OFILM			0x0018	// none (122)
#define		PWMPERIODX2_OFILM			0x0019	// none (122)
#define		PWMPERIODY_OFILM			0x001A	// PWMPERIODX (122)
#define		PWMPERIODY2_OFILM			0x001B	// PWMPERIODY (122)
#define		STROBEFC_OFILM			0x001C
#define		STROBEDLYX_OFILM			0x001D
#define		STROBEDLYY_OFILM			0x001E
//#define							0x001F
#define		CVA_OFILM					0x0020
#define		CVFC_OFILM				0x0021
#define		CVFC2_OFILM				0x0022
#define		CVSMTHX_OFILM				0x0023
#define		CVSMTHY_OFILM				0x0024
//#define							0x0025
//#define							0x0026
//#define							0x0027
//#define							0x0028
//#define							0x0029
//#define							0x002A
//#define							0x002B
//#define							0x002C
//#define							0x002D
//#define							0x002E
//#define							0x002F
#define		PWMMONA_OFILM				0x0030
#define		PWMMONFC_OFILM			0x0031
#define		DACMONFC_OFILM			0x0032
//#define							0x0033
//#define							0x0034
//#define							0x0035
//#define							0x0036
//#define							0x0037
//#define							0x0038
//#define							0x0039
//#define							0x003A
//#define							0x003B
//#define							0x003C
//#define							0x003D
//#define							0x003E
//#define							0x003F
#define		DACSLVADD_OFILM			0x0040
#define		DACMSTCODE_OFILM			0x0041
#define		DACFSCKRATE_OFILM			0x0042
#define		DACHSCKRATE_OFILM			0x0043
#define		DACI2CFC_OFILM			0x0044
#define		DACI2CA_OFILM				0x0045
//#define							0x0046
//#define							0x0047
//#define							0x0048
//#define							0x0049
//#define							0x004A
//#define							0x004B
//#define							0x004C
//#define							0x004D
//#define							0x004E
//#define							0x004F
//#define							0x0050
//#define							0x0051
//#define							0x0052
//#define							0x0053
//#define							0x0054
//#define							0x0055
//#define							0x0056
//#define							0x0057
//#define							0x0058
//#define							0x0059
//#define							0x005A
//#define							0x005B
//#define							0x005C
//#define							0x005D
//#define							0x005E
//#define							0x005F
//#define							0x0060
//#define							0x0061
//#define							0x0062
//#define							0x0063
//#define							0x0064
//#define							0x0065
//#define							0x0066
//#define							0x0067
//#define							0x0068
//#define							0x0069
//#define							0x006A
//#define							0x006B
//#define							0x006C
//#define							0x006D
//#define							0x006E
//#define							0x006F
//#define							0x0070
//#define							0x0071
//#define							0x0072
//#define							0x0073
//#define							0x0074
//#define							0x0075
//#define							0x0076
//#define							0x0077
//#define							0x0078
//#define							0x0079
//#define							0x007A
//#define							0x007B
//#define							0x007C
//#define							0x007D
//#define							0x007E
//#define							0x007F
//#define							0x0080
#define		DRVFCAF_OFILM				0x0081
#define		DRVFC2AF_OFILM			0x0082
#define		DRVFC3AF_OFILM			0x0083
#define		DRVFC4AF_OFILM			0x0084
#define		DRVCH3SEL_OFILM			0x0085
//#define							0x0086
//#define							0x0087
#define		AFFC_OFILM				0x0088
//#define							0x0089
//#define							0x008A
//#define							0x008B
//#define							0x008C
//#define							0x008D
//#define							0x008E
//#define							0x008F
#define		PWMAAF_OFILM				0x0090
#define		PWMFCAF_OFILM				0x0091
#define		PWMDLYAF_OFILM			0x0092
#define		PWMDLYTIMAF_OFILM			0x0093
//#define							0x0094
//#define							0x0095
//#define							0x0096
//#define							0x0097
//#define							0x0098
#define		PWMPERIODAF_OFILM			0x0099
//#define							0x009A
//#define							0x009B
//#define							0x009C
//#define							0x009D
//#define							0x009E
//#define							0x009F
#define		CCAAF_OFILM				0x00A0
#define		CCFCAF_OFILM				0x00A1
//#define							0x00A2
//#define							0x00A3
//#define							0x00A4
//#define							0x00A5
//#define							0x00A6
//#define							0x00A7
//#define							0x00A8
//#define							0x00A9
//#define							0x00AA
//#define							0x00AB
//#define							0x00AC
//#define							0x00AD
//#define							0x00AE
//#define							0x00AF
//#define							0x00B0
//#define							0x00B1
//#define							0x00B2
//#define							0x00B3
//#define							0x00B4
//#define							0x00B5
//#define							0x00B6
//#define							0x00B7
//#define							0x00B8
//#define							0x00B9
//#define							0x00BA
//#define							0x00BB
//#define							0x00BC
//#define							0x00BD
//#define							0x00BE
//#define							0x00BF
//#define							0x00C0
//#define							0x00C1
//#define							0x00C2
//#define							0x00C3
//#define							0x00C4
//#define							0x00C5
//#define							0x00C6
//#define							0x00C7
//#define							0x00C8
//#define							0x00C9
//#define							0x00CA
//#define							0x00CB
//#define							0x00CC
//#define							0x00CD
//#define							0x00CE
//#define							0x00CF
//#define							0x00D0
//#define							0x00D1
//#define							0x00D2
//#define							0x00D3
//#define							0x00D4
//#define							0x00D5
//#define							0x00D6
//#define							0x00D7
//#define							0x00D8
//#define							0x00D9
//#define							0x00DA
//#define							0x00DB
//#define							0x00DC
//#define							0x00DD
//#define							0x00DE
//#define							0x00DF
//#define							0x00E0
//#define							0x00E1
//#define							0x00E2
//#define							0x00E3
//#define							0x00E4
//#define							0x00E5
//#define							0x00E6
//#define							0x00E7
//#define							0x00E8
//#define							0x00E9
//#define							0x00EA
//#define							0x00EB
//#define							0x00EC
//#define							0x00ED
//#define							0x00EE
//#define							0x00EF
//#define							0x00F0
//#define							0x00F1
//#define							0x00F2
//#define							0x00F3
//#define							0x00F4
//#define							0x00F5
//#define							0x00F6
//#define							0x00F7
//#define							0x00F8
//#define							0x00F9
//#define							0x00FA
//#define							0x00FB
//#define							0x00FC
//#define							0x00FD
//#define							0x00FE
#define		MDLREG_OFILM				0x00FF


//==============================================================================
//Filter Register
//==============================================================================
//#define							0x0100
#define		WC_EQON_OFILM				0x0101
#define		WC_RAMINITON_OFILM		0x0102
#define		WC_CPUOPEON_OFILM			0x0103
#define		WC_VMON_OFILM				0x0104
#define		WC_DPON_OFILM				0x0105
//#define							0x0106
#define		WG_SHTON_OFILM			0x0107
#define		WG_ADJGANGO_OFILM			0x0108
#define		WG_PANON_OFILM			0x0109
#define		WG_PANSTT6_OFILM			0x010A
#define		WG_NPANSTFRC_OFILM		0x010B
#define		WG_CNTPICGO_OFILM			0x010C
#define		WG_NPANINION_OFILM		0x010D
#define		WG_NPANSTOFF_OFILM		0x010E
//#define							0x010F
#define		WG_EQSW_OFILM				0x0110
#define		WG_DWNSMP1_OFILM			0x0111
#define		WG_DWNSMP2_OFILM			0x0112
#define		WG_DWNSMP3_OFILM			0x0113
#define		WG_DWNSMP4_OFILM			0x0114
//#define							0x0115
#define		WG_SHTMOD_OFILM			0x0116
#define		WG_SHTDLYTMR_OFILM		0x0117
#define		WG_LMT3MOD_OFILM			0x0118
#define		WG_VREFADD_OFILM			0x0119
//#define							0x011A
#define		WG_HCHR_OFILM				0x011B
#define		WG_GADSMP_OFILM			0x011C
//#define							0x011D
//#define							0x011E
//#define							0x011F
#define		WG_LEVADD_OFILM			0x0120
#define		WG_LEVTMRLOW_OFILM		0x0121
#define		WG_LEVTMRHGH_OFILM		0x0122
#define		WG_LEVTMR_OFILM			0x0123
//#define							0x0124
//#define							0x0125
//#define							0x0126
//#define							0x0127
#define		WG_ADJGANADD_OFILM		0x0128
#define		WG_ADJGANGXATO_OFILM		0x0129
#define		WG_ADJGANGYATO_OFILM		0x012A
//#define							0x012B
//#define							0x012C
//#define							0x012D
//#define							0x012E
//#define							0x012F
#define		WG_PANADDA_OFILM			0x0130
#define		WG_PANADDB_OFILM			0x0131
#define		WG_PANTRSON0_OFILM		0x0132
#define		WG_PANLEVABS_OFILM		0x0133
#define		WG_PANSTT1DWNSMP0_OFILM	0x0134
#define		WG_PANSTT1DWNSMP1_OFILM	0x0135
#define		WG_PANSTT2DWNSMP0_OFILM	0x0136
#define		WG_PANSTT2DWNSMP1_OFILM	0x0137
#define		WG_PANSTT3DWNSMP0_OFILM	0x0138
#define		WG_PANSTT3DWNSMP1_OFILM	0x0139
#define		WG_PANSTT4DWNSMP0_OFILM	0x013A
#define		WG_PANSTT4DWNSMP1_OFILM	0x013B
#define		WG_PANSTT2TMR0_OFILM		0x013C
#define		WG_PANSTT2TMR1_OFILM		0x013D
#define		WG_PANSTT4TMR0_OFILM		0x013E
#define		WG_PANSTT4TMR1_OFILM		0x013F
#define		WG_PANSTT21JUG0_OFILM		0x0140
#define		WG_PANSTT21JUG1_OFILM		0x0141
#define		WG_PANSTT31JUG0_OFILM		0x0142
#define		WG_PANSTT31JUG1_OFILM		0x0143
#define		WG_PANSTT41JUG0_OFILM		0x0144
#define		WG_PANSTT41JUG1_OFILM		0x0145
#define		WG_PANSTT12JUG0_OFILM		0x0146
#define		WG_PANSTT12JUG1_OFILM		0x0147
#define		WG_PANSTT13JUG0_OFILM		0x0148
#define		WG_PANSTT13JUG1_OFILM		0x0149
#define		WG_PANSTT23JUG0_OFILM		0x014A
#define		WG_PANSTT23JUG1_OFILM		0x014B
#define		WG_PANSTT43JUG0_OFILM		0x014C
#define		WG_PANSTT43JUG1_OFILM		0x014D
#define		WG_PANSTT34JUG0_OFILM		0x014E
#define		WG_PANSTT34JUG1_OFILM		0x014F
#define		WG_PANSTT24JUG0_OFILM		0x0150
#define		WG_PANSTT24JUG1_OFILM		0x0151
#define		WG_PANSTT42JUG0_OFILM		0x0152
#define		WG_PANSTT42JUG1_OFILM		0x0153
#define		WG_PANSTTSETGYRO_OFILM	0x0154
#define		WG_PANSTTSETGAIN_OFILM	0x0155
#define		WG_PANSTTSETISTP_OFILM	0x0156
#define		WG_PANSTTSETIFTR_OFILM	0x0157
#define		WG_PANSTTSETLFTR_OFILM	0x0158
//#define							0x0159
#define		WG_PANSTTXXXTH_OFILM		0x015A
#define		WG_PANSTT1LEVTMR_OFILM	0x015B
#define		WG_PANSTT2LEVTMR_OFILM	0x015C
#define		WG_PANSTT3LEVTMR_OFILM	0x015D
#define		WG_PANSTT4LEVTMR_OFILM	0x015E
#define		WG_PANSTTSETILHLD_OFILM	0x015F
#define		WG_STT3MOD_OFILM			0x0160
#define		WG_STILMOD_OFILM			0x0161
#define		WG_PLAYON_OFILM			0x0162
#define		WG_NPANJ2DWNSMP_OFILM		0x0163
#define		WG_NPANTST0_OFILM			0x0164
#define		WG_NPANDWNSMP_OFILM		0x0165
#define		WG_NPANST3RTMR_OFILM		0x0166
#define		WG_NPANST12BTMR_OFILM		0x0167
#define		WG_NPANST12TMRX_OFILM		0x0168
#define		WG_NPANST12TMRY_OFILM		0x0169
#define		WG_NPANST3TMRX_OFILM		0x016A
#define		WG_NPANST3TMRY_OFILM		0x016B
#define		WG_NPANST4TMRX_OFILM		0x016C
#define		WG_NPANST4TMRY_OFILM		0x016D
#define		WG_NPANFUN_OFILM			0x016E
#define		WG_NPANINITMR_OFILM		0x016F
#define		WH_EQSWX_OFILM			0x0170
#define		WH_EQSWY_OFILM			0x0171
	#define		EQSINSW_OFILM				0x3C
#define		WH_DWNSMP1_OFILM			0x0172
#define		WH_G2SDLY_OFILM			0x0173
#define		WH_HOFCON_OFILM			0x0174
//#define							0x0175
//#define							0x0176
//#define							0x0177
#define		WH_EMGSTPON_OFILM			0x0178
//#define							0x0179
#define		WH_EMGSTPTMR_OFILM		0x017A
//#define							0x017B
#define		WH_SMTSRVON_OFILM			0x017C
#define		WH_SMTSRVSMP_OFILM		0x017D
#define		WH_SMTTMR_OFILM			0x017E
//#define							0x017F
#define		WC_SINON_OFILM			0x0180
#define		WC_SINFRQ0_OFILM			0x0181
#define		WC_SINFRQ1_OFILM			0x0182
#define		WC_SINPHSX_OFILM			0x0183
#define		WC_SINPHSY_OFILM			0x0184
//#define							0x0185
//#define							0x0186
//#define							0x0187
#define		WC_ADMODE_OFILM			0x0188
//#define							0x0189
#define		WC_CPUOPE1ADD_OFILM		0x018A
#define		WC_CPUOPE2ADD_OFILM		0x018B
#define		WC_RAMACCMOD_OFILM		0x018C
#define		WC_RAMACCXY_OFILM			0x018D
#define		WC_RAMDLYMOD0_OFILM		0x018E
#define		WC_RAMDLYMOD1_OFILM		0x018F
#define		WC_MESMODE_OFILM			0x0190
#define		WC_MESSINMODE_OFILM		0x0191
#define		WC_MESLOOP0_OFILM			0x0192
#define		WC_MESLOOP1_OFILM			0x0193
#define		WC_MES1ADD0_OFILM			0x0194
#define		WC_MES1ADD1_OFILM			0x0195
#define		WC_MES2ADD0_OFILM			0x0196
#define		WC_MES2ADD1_OFILM			0x0197
#define		WC_MESABS_OFILM			0x0198
#define		WC_MESWAIT_OFILM			0x0199
//#define							0x019A
//#define							0x019B
//#define							0x019C
#define		RC_MESST_OFILM			0x019D
#define		RC_MESLOOP0_OFILM			0x019E
#define		RC_MESLOOP1_OFILM			0x019F
#define		WC_AMJMODE_OFILM			0x01A0
#define		WC_AMJDF_OFILM			0x01A1
#define		WC_AMJLOOP0_OFILM			0x01A2
#define		WC_AMJLOOP1_OFILM			0x01A3
#define		WC_AMJIDL0_OFILM			0x01A4
#define		WC_AMJIDL1_OFILM			0x01A5
#define		WC_AMJ1ADD0_OFILM			0x01A6
#define		WC_AMJ1ADD1_OFILM			0x01A7
#define		WC_AMJ2ADD0_OFILM			0x01A8
#define		WC_AMJ2ADD1_OFILM			0x01A9
//#define							0x01AA
//#define							0x01AB
#define		RC_AMJST_OFILM			0x01AC
#define		RC_AMJERROR_OFILM			0x01AD
#define		RC_AMJLOOP0_OFILM			0x01AE
#define		RC_AMJLOOP1_OFILM			0x01AF
#define		WC_DPI1ADD0_OFILM			0x01B0
#define		WC_DPI1ADD1_OFILM			0x01B1
#define		WC_DPI2ADD0_OFILM			0x01B2
#define		WC_DPI2ADD1_OFILM			0x01B3
#define		WC_DPI3ADD0_OFILM			0x01B4
#define		WC_DPI3ADD1_OFILM			0x01B5
#define		WC_DPI4ADD0_OFILM			0x01B6
#define		WC_DPI4ADD1_OFILM			0x01B7
#define		WC_DPO1ADD0_OFILM			0x01B8
#define		WC_DPO1ADD1_OFILM			0x01B9
#define		WC_DPO2ADD0_OFILM			0x01BA
#define		WC_DPO2ADD1_OFILM			0x01BB
#define		WC_DPO3ADD0_OFILM			0x01BC
#define		WC_DPO3ADD1_OFILM			0x01BD
#define		WC_DPO4ADD0_OFILM			0x01BE
#define		WC_DPO4ADD1_OFILM			0x01BF
#define		WC_PINMON1_OFILM			0x01C0
#define		WC_PINMON2_OFILM			0x01C1
#define		WC_PINMON3_OFILM			0x01C2
#define		WC_PINMON4_OFILM			0x01C3
#define		WC_DLYMON10_OFILM			0x01C4
#define		WC_DLYMON11_OFILM			0x01C5
#define		WC_DLYMON20_OFILM			0x01C6
#define		WC_DLYMON21_OFILM			0x01C7
#define		WC_DLYMON30_OFILM			0x01C8
#define		WC_DLYMON31_OFILM			0x01C9
#define		WC_DLYMON40_OFILM			0x01CA
#define		WC_DLYMON41_OFILM			0x01CB
//#define							0x01CC
//#define							0x01CD
#define		WC_INTMSK_OFILM			0x01CE
//#define							0x01CF
#define		WC_FRCAD_OFILM			0x01D0
#define		WC_FRCADEN_OFILM			0x01D1
#define		WC_ADRES_OFILM			0x01D2
#define		WC_TSTMON_OFILM			0x01D3
#define		WC_RAMACCTM0_OFILM		0x01D4
#define		WC_RAMACCTM1_OFILM		0x01D5
//#define							0x01D6
//#define							0x01D7
//#define							0x01D8
//#define							0x01D9
//#define							0x01DA
//#define							0x01DB
//#define							0x01DC
//#define							0x01DD
//#define							0x01DE
//#define							0x01DF
#define		WC_EQSW_OFILM				0x01E0
#define		WC_STPMV_OFILM			0x01E1
#define		WC_STPMVMOD_OFILM			0x01E2
#define		WC_DWNSMP1_OFILM			0x01E3
#define		WC_DWNSMP2_OFILM			0x01E4
#define		WC_DWNSMP3_OFILM			0x01E5
#define		WC_LEVTMP_OFILM			0x01E6
#define		WC_DIFTMP_OFILM			0x01E7
#define		WC_L10_OFILM				0x01E8
#define		WC_L11_OFILM				0x01E9
//#define							0x01EA
//#define							0x01EB
//#define							0x01EC
//#define							0x01ED
//#define							0x01EE
//#define							0x01EF
#define		RG_XPANFIL_OFILM			0x01F0
#define		RG_YPANFIL_OFILM			0x01F1
#define		RG_XPANRAW_OFILM			0x01F2
#define		RG_YPANRAW_OFILM			0x01F3
#define		RG_LEVJUGE_OFILM			0x01F4
#define		RG_NXPANST_OFILM			0x01F5
#define		RC_RAMACC_OFILM			0x01F6
#define		RH_EMLEV_OFILM			0x01F7
#define		RH_SMTSRVSTT_OFILM		0x01F8
#define		RC_CNTPIC_OFILM			0x01F9
#define		RC_LEVDIF_OFILM			0x01FA
//#define							0x01FB
//#define							0x01FC
//#define							0x01FD
#define		RC_FLG0_OFILM				0x01FE
#define		RC_INT_OFILM				0x01FF


//==============================================================================
//System Register
//==============================================================================
//#define							0x0200
//#define							0x0201
//#define							0x0202
//#define							0x0203
//#define							0x0204
//#define							0x0205
//#define							0x0206
//#define							0x0207
//#define							0x0208
//#define							0x0209
#define		CLKTST_OFILM				0x020A
#define		CLKON_OFILM				0x020B
#define		CLKSEL_OFILM				0x020C
//#define							0x020D
//#define							0x020E
//#define							0x020F
#define		PWMDIV_OFILM				0x0210
#define		SRVDIV_OFILM				0x0211
#define		GIFDIV_OFILM				0x0212
#define		AFPWMDIV_OFILM			0x0213
#define		OPAFDIV_OFILM				0x0214
//#define							0x0215
//#define							0x0216
//#define							0x0217
//#define							0x0218
//#define							0x0219
//#define							0x021A
//#define							0x021B
//#define							0x021C
//#define							0x021D
//#define							0x021E
//#define							0x021F
#define		P0LEV_OFILM				0x0220
#define		P0DIR_OFILM				0x0221
#define		P0PON_OFILM				0x0222
#define		P0PUD_OFILM				0x0223
//#define							0x0224
//#define							0x0225
//#define							0x0226
//#define							0x0227
//#define							0x0228
//#define							0x0229
//#define							0x022A
//#define							0x022B
//#define							0x022C
//#define							0x022D
//#define							0x022E
//#define							0x022F
#define		IOP0SEL_OFILM				0x0230
#define		IOP1SEL_OFILM				0x0231
#define		IOP2SEL_OFILM				0x0232
#define		IOP3SEL_OFILM				0x0233
#define		IOP4SEL_OFILM				0x0234
#define		IOP5SEL_OFILM				0x0235
#define		DGINSEL_OFILM				0x0236
//#define							0x0237
#define		IOP_CNT_OFILM				0x0238
#define		OUT56MON_OFILM			0x0239
//#define							0x023A
//#define							0x023B
//#define							0x023C
//#define							0x023D
//#define							0x023E
//#define							0x023F
#define		BSYSEL_OFILM				0x0240
//#define							0x0241
//#define							0x0242
//#define							0x0243
//#define							0x0244
//#define							0x0245
//#define							0x0246
//#define							0x0247
#define		I2CSEL_OFILM				0x0248
#define		DLMODE_OFILM				0x0249
//#define							0x024A
//#define							0x024B
//#define							0x024C
//#define							0x024D
#define		TSTREG0_OFILM				0x024E
#define		TSTREG1_OFILM				0x024F
#define		STBB0_OFILM				0x0250
#define		CMSDAC0_OFILM				0x0251
#define		CMSDAC1_OFILM				0x0252
#define		OPGSEL0_OFILM				0x0253
#define		OPGSEL1_OFILM				0x0254
#define		OPGSEL2_OFILM				0x0255
#define		OSCSTOP_OFILM				0x0256
#define		OSCSET_OFILM				0x0257
#define		OSCCNTEN_OFILM			0x0258
#define		LDO_C_SET_OFILM			0x0259
#define		VGA_SW0_OFILM				0x025A
#define		VGA_SW1_OFILM				0x025B
#define		RSTRLSCNTL_OFILM			0x025C
#define		RSTRLSCNTH_OFILM			0x025D
#define		OSCCK_CNTR0_OFILM			0x025E
#define		OSCCK_CNTR1_OFILM			0x025F
#define		EXTCNTEN_OFILM			0x0260
#define		EXTCLKLOW_OFILM			0x0261
#define		ADCTEST_OFILM				0x0262
#define		LDSTB_OFILM				0x0263
#define		STBB1_OFILM				0x0264
//#define							0x0265
//#define							0x0266
//#define							0x0267
//#define							0x0268
//#define							0x0269
//#define							0x026A
//#define							0x026B
//#define							0x026C
//#define							0x026D
//#define							0x026E
//#define							0x026F
#define		MONSELA_OFILM				0x0270
#define		MONSELB_OFILM				0x0271
#define		MONSELC_OFILM				0x0272
#define		MONSELD_OFILM				0x0273
#define		CmMonTst_OFILM			0x0274
//#define							0x0275
//#define							0x0276
//#define							0x0277
#define		SOFTRES1_OFILM			0x0278
#define		SOFTRES2_OFILM			0x0279
//#define							0x027A
//#define							0x027B
//#define							0x027C
//#define							0x027D
#define		CVER_OFILM				0x027E
#define		TESTRD_OFILM				0x027F


//==============================================================================
//Digital Gyro I/F Register
//==============================================================================
#define		GRSEL_OFILM				0x0280
#define		GRINI_OFILM				0x0281
	#define		SLOWMODE_OFILM			0x04			/* 0:4MHz	1:1MHz	*/
#define		GRACC_OFILM				0x0282
#define		GRADR0_OFILM				0x0283
#define		GRADR1_OFILM				0x0284
#define		GRADR2_OFILM				0x0285
#define		GRADR3_OFILM				0x0286
#define		GRADR4_OFILM				0x0287
#define		GRADR5_OFILM				0x0288
#define		GRADR6_OFILM				0x0289
#define		GSETDT_OFILM				0x028A
#define		RDSEL_OFILM				0x028B
#define		REVB7_OFILM				0x028C
#define		LSBF_OFILM				0x028D
#define		PANAM_OFILM				0x028E
#define		SPIM_OFILM				0x028F
#define		GRDAT0H_OFILM				0x0290
#define		GRDAT0L_OFILM				0x0291
#define		GRDAT1H_OFILM				0x0292
#define		GRDAT1L_OFILM				0x0293
#define		GRDAT2H_OFILM				0x0294
#define		GRDAT2L_OFILM				0x0295
#define		GRDAT3H_OFILM				0x0296
#define		GRDAT3L_OFILM				0x0297
#define		GRDAT4H_OFILM				0x0298
#define		GRDAT4L_OFILM				0x0299
#define		GRDAT5H_OFILM				0x029A
#define		GRDAT5L_OFILM				0x029B
#define		GRDAT6H_OFILM				0x029C
#define		GRDAT6L_OFILM				0x029D
//#define							0x029E
//#define							0x029F
#define		IZAH_OFILM				0x02A0
#define		IZAL_OFILM				0x02A1
#define		IZBH_OFILM				0x02A2
#define		IZBL_OFILM				0x02A3
//#define							0x02A4
//#define							0x02A5
//#define							0x02A6
//#define							0x02A7
//#define							0x02A8
//#define							0x02A9
//#define							0x02AA
//#define							0x02AB
//#define							0x02AC
//#define							0x02AD
//#define							0x02AE
//#define							0x02AF
//#define							0x02B0
//#define							0x02B1
//#define							0x02B2
//#define							0x02B3
//#define							0x02B4
//#define							0x02B5
//#define							0x02B6
//#define							0x02B7
#define		GRFLG0_OFILM				0x02B8
#define		GRFLG1_OFILM				0x02B9
//#define							0x02BA
//#define							0x02BB
//#define							0x02BC
//#define							0x02BD
//#define							0x02BE
//#define							0x02BF
//#define							0x02C0
#define		DGSTAT0_OFILM				0x02C1
#define		DGSTAT1_OFILM				0x02C2
//#define							0x02C3
//#define							0x02C4
//#define							0x02C5
//#define							0x02C6
//#define							0x02C7
//#define							0x02C8
//#define							0x02C9
//#define							0x02CA
//#define							0x02CB
//#define							0x02CC
//#define							0x02CD
//#define							0x02CE
//#define							0x02CF
#define		VRREG_OFILM				0x02D0			// USE TEST REG
//#define							0x02D1
//#define							0x02D2
//#define							0x02D3
//#define							0x02D4
//#define							0x02D5
//#define							0x02D6
//#define							0x02D7
//#define							0x02D8
//#define							0x02D9
//#define							0x02DA
//#define							0x02DB
//#define							0x02DC
//#define							0x02DD
//#define							0x02DE
//#define							0x02DF
//#define							0x02E0
//#define							0x02E1
//#define							0x02E2
//#define							0x02E3
//#define							0x02E4
//#define							0x02E5
//#define							0x02E6
//#define							0x02E7
//#define							0x02E8
//#define							0x02E9
//#define							0x02EA
//#define							0x02EB
//#define							0x02EC
//#define							0x02ED
//#define							0x02EE
//#define							0x02EF
//#define							0x02F0
//#define							0x02F1
//#define							0x02F2
//#define							0x02F3
//#define							0x02F4
//#define							0x02F5
//#define							0x02F6
//#define							0x02F7
//#define							0x02F8
//#define							0x02F9
//#define							0x02FA
//#define							0x02FB
//#define							0x02FC
//#define							0x02FD
//#define							0x02FE
//#define							0x02FF


//==============================================================================
//Open AF Register
//==============================================================================
//#define							0x0300
//#define							0x0301
#define		FSTMODE_OFILM				0x0302
#define		FSTCTIME_OFILM			0x0303
#define		TCODEH_OFILM				0x0304
#define		TCODEL_OFILM				0x0305
#define		LTHDH_OFILM				0x0306
#define		LTHDL_OFILM				0x0307
//#define							0x0308
//#define							0x0309
//#define							0x030A
//#define							0x030B
//#define							0x030C
//#define							0x030D
//#define							0x030E
//#define							0x030F
#define		FSTOPTION_OFILM			0x0310
//#define							0x0311
//#define							0x0312
//#define							0x0313
//#define							0x0314
//#define							0x0315
//#define							0x0316
//#define							0x0317
//#define							0x0318
//#define							0x0319
//#define							0x031A
//#define							0x031B
//#define							0x031C
//#define							0x031D
//#define							0x031E
//#define							0x031F
#define		OPAFEN_OFILM				0x0320
//#define							0x0321
//#define							0x0322
//#define							0x0323
//#define							0x0324
//#define							0x0325
//#define							0x0326
//#define							0x0327
//#define							0x0328
//#define							0x0329
//#define							0x032A
//#define							0x032B
//#define							0x032C
//#define							0x032D
//#define							0x032E
//#define							0x032F
#define		OPAFSW_OFILM				0x0330
//#define							0x0331
//#define							0x0332
//#define							0x0333
//#define							0x0334
#define		OPAFST_OFILM				0x0335

//#define							0x0380
//#define							0x0381
//#define							0x0382
//#define							0x0383
//#define							0x0384
//#define							0x0385
//#define							0x0386
//#define							0x0387
//#define							0x0388
//#define							0x0389
//#define							0x038A
//#define							0x038B
//#define							0x038C
//#define							0x038D
//#define							0x038E
//#define							0x038F
//#define							0x0390
//#define							0x0391
//#define							0x0392
//#define							0x0393
//#define							0x0394
//#define							0x0395
#define		RWEXD1_L_OFILM			0x0396		// 2Byte access
//#define							0x0397
#define		RWEXD2_L_OFILM			0x0398		// 2Byte access
//#define							0x0399
#define		RWEXD3_L_OFILM			0x039A		// 2Byte access
//#define							0x039B
//#define							0x039C
//#define							0x039D
//#define							0x039E
//#define							0x039F

//==============================================================================
//FILTER COEFFICIENT RAM
//==============================================================================
#define		gx45g_OFILM				0x1000
#define		gx45x_OFILM				0x1001
#define		gx45y_OFILM				0x1002
#define		gxgyro_OFILM				0x1003
#define		gxia_OFILM				0x1004
#define		gxib_OFILM				0x1005
#define		gxic_OFILM				0x1006
#define		gxggain_OFILM				0x1007
#define		gxigain_OFILM				0x1008
#define		gxggain2_OFILM			0x1009
#define		gx2x4xf_OFILM				0x100A
#define		gxadj_OFILM				0x100B
#define		gxgain_OFILM				0x100C
#define		gxl3_OFILM				0x100D
#define		gxhc_tmp_OFILM			0x100E
#define		npxlev1_OFILM				0x100F
#define		gxh1a_OFILM				0x1010
#define		gxh1b_OFILM				0x1011
#define		gxh1c_OFILM				0x1012
#define		gxh2a_OFILM				0x1013
#define		gxh2b_OFILM				0x1014
#define		gxh2c_OFILM				0x1015
#define		gxh3a_OFILM				0x1016
#define		gxh3b_OFILM				0x1017
#define		gxh3c_OFILM				0x1018
#define		gxla_OFILM				0x1019
#define		gxlb_OFILM				0x101A
#define		gxlc_OFILM				0x101B
#define		gxhgain_OFILM				0x101C
#define		gxlgain_OFILM				0x101D
#define		gxigainstp_OFILM			0x101E
#define		npxlev2_OFILM				0x101F
#define		gxzoom_OFILM				0x1020
#define		gx2x4xb_OFILM				0x1021
#define		gxlens_OFILM				0x1022
#define		gxta_OFILM				0x1023
#define		gxtb_OFILM				0x1024
#define		gxtc_OFILM				0x1025
#define		gxtd_OFILM				0x1026
#define		gxte_OFILM				0x1027
#define		gxlmt1H_OFILM				0x1028
#define		gxlmt3HS0_OFILM			0x1029
#define		gxlmt3HS1_OFILM			0x102A
#define		gxlmt4HS0_OFILM			0x102B
#define		gxlmt4HS1_OFILM			0x102C
#define		gxlmt6L_OFILM				0x102D
#define		gxlmt6H_OFILM				0x102E
#define		npxlev3_OFILM				0x102F
#define		gxj1a_OFILM				0x1030
#define		gxj1b_OFILM				0x1031
#define		gxj1c_OFILM				0x1032
#define		gxj2a_OFILM				0x1033
#define		gxj2b_OFILM				0x1034
#define		gxj2c_OFILM				0x1035
#define		gxk1a_OFILM				0x1036
#define		gxk1b_OFILM				0x1037
#define		gxk1c_OFILM				0x1038
#define		gxk2a_OFILM				0x1039
#define		gxk2b_OFILM				0x103A
#define		gxk2c_OFILM				0x103B
#define		gxoa_OFILM				0x103C
#define		gxob_OFILM				0x103D
#define		gxoc_OFILM				0x103E
#define		npxlev4_OFILM				0x103F
#define		MSABS1_OFILM				0x1040
#define		MSABS1AV_OFILM			0x1041
#define		MSPP1AV_OFILM				0x1042
#define		gxia_1_OFILM				0x1043
#define		gxib_1_OFILM				0x1044
#define		gxic_1_OFILM				0x1045
#define		gxia_a_OFILM				0x1046
#define		gxib_a_OFILM				0x1047
#define		gxic_a_OFILM				0x1048
#define		gxia_b_OFILM				0x1049
#define		gxib_b_OFILM				0x104A
#define		gxic_b_OFILM				0x104B
#define		gxia_c_OFILM				0x104C
#define		gxib_c_OFILM				0x104D
#define		gxic_c_OFILM				0x104E
#define		Sttx12aM_OFILM			0x104F
#define		MSMAX1_OFILM				0x1050
#define		MSMAX1AV_OFILM			0x1051
#define		MSCT1AV_OFILM				0x1052
#define		gxla_1_OFILM				0x1053
#define		gxlb_1_OFILM				0x1054
#define		gxlc_1_OFILM				0x1055
#define		gxla_a_OFILM				0x1056
#define		gxlb_a_OFILM				0x1057
#define		gxlc_a_OFILM				0x1058
#define		gxla_b_OFILM				0x1059
#define		gxlb_b_OFILM				0x105A
#define		gxlc_b_OFILM				0x105B
#define		gxla_c_OFILM				0x105C
#define		gxlb_c_OFILM				0x105D
#define		gxlc_c_OFILM				0x105E
#define		Sttx12aH_OFILM			0x105F
#define		MSMIN1_OFILM				0x1060
#define		MSMIN1AV_OFILM			0x1061
#define		MS1AV_OFILM				0x1062
#define		gxgyro_1_OFILM			0x1063
#define		gxgyro_1d_OFILM			0x1064
#define		gxgyro_1u_OFILM			0x1065
#define		gxgyro_a_OFILM			0x1066
#define		gxgyro_2d_OFILM			0x1067
#define		gxgyro_2u_OFILM			0x1068
#define		gxgyro_b_OFILM			0x1069
#define		gxgyro_3d_OFILM			0x106A
#define		gxgyro_3u_OFILM			0x106B
#define		gxgyro_c_OFILM			0x106C
#define		gxgyro_4d_OFILM			0x106D
#define		gxgyro_4u_OFILM			0x106E
#define		Sttx12bM_OFILM			0x106F
#define		HOStp_OFILM				0x1070
#define		HOMin_OFILM				0x1071
#define		HOMax_OFILM				0x1072
#define		gxgain_1_OFILM			0x1073
#define		gxgain_1d_OFILM			0x1074
#define		gxgain_1u_OFILM			0x1075
#define		gxgain_a_OFILM			0x1076
#define		gxgain_2d_OFILM			0x1077
#define		gxgain_2u_OFILM			0x1078
#define		gxgain_b_OFILM			0x1079
#define		gxgain_3d_OFILM			0x107A
#define		gxgain_3u_OFILM			0x107B
#define		gxgain_c_OFILM			0x107C
#define		gxgain_4d_OFILM			0x107D
#define		gxgain_4u_OFILM			0x107E
#define		Sttx12bH_OFILM			0x107F
#define		HBStp_OFILM				0x1080
#define		HBMin_OFILM				0x1081
#define		HBMax_OFILM				0x1082
#define		gxistp_1_OFILM			0x1083
#define		gxistp_1d_OFILM			0x1084
#define		gxistp_1u_OFILM			0x1085
#define		gxistp_a_OFILM			0x1086
#define		gxistp_2d_OFILM			0x1087
#define		gxistp_2u_OFILM			0x1088
#define		gxistp_b_OFILM			0x1089
#define		gxistp_3d_OFILM			0x108A
#define		gxistp_3u_OFILM			0x108B
#define		gxistp_c_OFILM			0x108C
#define		gxistp_4d_OFILM			0x108D
#define		gxistp_4u_OFILM			0x108E
#define		Sttx34aM_OFILM			0x108F
#define		LGStp_OFILM				0x1090
#define		LGMin_OFILM				0x1091
#define		LGMax_OFILM				0x1092
#define		gxistp_OFILM				0x1093
#define		gxadjmin_OFILM			0x1094
#define		gxadjmax_OFILM			0x1095
#define		gxadjdn_OFILM				0x1096
#define		gxadjup_OFILM				0x1097
#define		gxog3_OFILM				0x1098
#define		gxog5_OFILM				0x1099
#define		gxog7_OFILM				0x109A
#define		npxlev8_OFILM				0x109B
#define		sxlmtb1_OFILM				0x109C
#define		SttxaL_OFILM				0x109D
#define		SttxbL_OFILM				0x109E
#define		Sttx34aH_OFILM			0x109F
#define		sxlmtb2_OFILM				0x10A0
#define		pxmaa_OFILM				0x10A1
#define		pxmab_OFILM				0x10A2
#define		pxmac_OFILM				0x10A3
#define		pxmba_OFILM				0x10A4
#define		pxmbb_OFILM				0x10A5
#define		pxmbc_OFILM				0x10A6
#define		gxma_OFILM				0x10A7
#define		gxmb_OFILM				0x10A8
#define		gxmc_OFILM				0x10A9
#define		gxmg_OFILM				0x10AA
#define		gxleva_OFILM				0x10AB
#define		gxlevb_OFILM				0x10AC
#define		gxlevc_OFILM				0x10AD
#define		gxlevlow_OFILM			0x10AE
#define		Sttx34bM_OFILM			0x10AF
#define		sxria_OFILM				0x10B0
#define		sxrib_OFILM				0x10B1
#define		sxric_OFILM				0x10B2
#define		sxinx_OFILM				0x10B3
#define		sxiny_OFILM				0x10B4
#define		sxggf_OFILM				0x10B5
#define		sxag_OFILM				0x10B6
#define		sxpr_OFILM				0x10B7
#define		sxgx_OFILM				0x10B8
#define		sxgy_OFILM				0x10B9
#define		sxiexp3_OFILM				0x10BA
#define		sxiexp2_OFILM				0x10BB
#define		sxiexp1_OFILM				0x10BC
#define		sxiexp0_OFILM				0x10BD
#define		sxiexp_OFILM				0x10BE
#define		Sttx34bH_OFILM			0x10BF
#define		sxda_OFILM				0x10C0
#define		sxdb_OFILM				0x10C1
#define		sxdc_OFILM				0x10C2
#define		sxea_OFILM				0x10C3
#define		sxeb_OFILM				0x10C4
#define		sxec_OFILM				0x10C5
#define		sxua_OFILM				0x10C6
#define		sxub_OFILM				0x10C7
#define		sxuc_OFILM				0x10C8
#define		sxia_OFILM				0x10C9
#define		sxib_OFILM				0x10CA
#define		sxic_OFILM				0x10CB
#define		sxja_OFILM				0x10CC
#define		sxjb_OFILM				0x10CD
#define		sxjc_OFILM				0x10CE
#define		npxlev1_i_OFILM			0x10CF
#define		sxfa_OFILM				0x10D0
#define		sxfb_OFILM				0x10D1
#define		sxfc_OFILM				0x10D2
#define		sxg_OFILM					0x10D3
#define		sxg2_OFILM				0x10D4
#define		sxsin_OFILM				0x10D5
#define		sxggf_tmp_OFILM			0x10D6
#define		sxsa_OFILM				0x10D7
#define		sxsb_OFILM				0x10D8
#define		sxsc_OFILM				0x10D9
#define		sxoa_OFILM				0x10DA
#define		sxob_OFILM				0x10DB
#define		sxoc_OFILM				0x10DC
#define		sxod_OFILM				0x10DD
#define		sxoe_OFILM				0x10DE
#define		npxlev2_i_OFILM			0x10DF
#define		sxpa_OFILM				0x10E0
#define		sxpb_OFILM				0x10E1
#define		sxpc_OFILM				0x10E2
#define		sxpd_OFILM				0x10E3
#define		sxpe_OFILM				0x10E4
#define		sxq_OFILM					0x10E5
#define		sxlmta1_OFILM				0x10E6
#define		sxlmta2_OFILM				0x10E7
#define		smxga_OFILM				0x10E8
#define		smxgb_OFILM				0x10E9
#define		smxa_OFILM				0x10EA
#define		smxb_OFILM				0x10EB
#define		sxemglev_OFILM			0x10EC
#define		sxsmtav_OFILM				0x10ED
#define		sxsmtstp_OFILM			0x10EE
#define		npxlev3_i_OFILM			0x10EF
#define		mes1aa_OFILM				0x10F0
#define		mes1ab_OFILM				0x10F1
#define		mes1ac_OFILM				0x10F2
#define		mes1ad_OFILM				0x10F3
#define		mes1ae_OFILM				0x10F4
#define		mes1ba_OFILM				0x10F5
#define		mes1bb_OFILM				0x10F6
#define		mes1bc_OFILM				0x10F7
#define		mes1bd_OFILM				0x10F8
#define		mes1be_OFILM				0x10F9
#define		sxoexp3_OFILM				0x10FA
#define		sxoexp2_OFILM				0x10FB
#define		sxoexp1_OFILM				0x10FC
#define		sxoexp0_OFILM				0x10FD
#define		sxoexp_OFILM				0x10FE
#define		npxlev4_i_OFILM			0x10FF
#define		gy45g_OFILM				0x1100
#define		gy45y_OFILM				0x1101
#define		gy45x_OFILM				0x1102
#define		gygyro_OFILM				0x1103
#define		gyia_OFILM				0x1104
#define		gyib_OFILM				0x1105
#define		gyic_OFILM				0x1106
#define		gyggain_OFILM				0x1107
#define		gyigain_OFILM				0x1108
#define		gyggain2_OFILM			0x1109
#define		gy2x4xf_OFILM				0x110A
#define		gyadj_OFILM				0x110B
#define		gygain_OFILM				0x110C
#define		gyl3_OFILM				0x110D
#define		gyhc_tmp_OFILM			0x110E
#define		npylev1_OFILM				0x110F
#define		gyh1a_OFILM				0x1110
#define		gyh1b_OFILM				0x1111
#define		gyh1c_OFILM				0x1112
#define		gyh2a_OFILM				0x1113
#define		gyh2b_OFILM				0x1114
#define		gyh2c_OFILM				0x1115
#define		gyh3a_OFILM				0x1116
#define		gyh3b_OFILM				0x1117
#define		gyh3c_OFILM				0x1118
#define		gyla_OFILM				0x1119
#define		gylb_OFILM				0x111A
#define		gylc_OFILM				0x111B
#define		gyhgain_OFILM				0x111C
#define		gylgain_OFILM				0x111D
#define		gyigainstp_OFILM			0x111E
#define		npylev2_OFILM				0x111F
#define		gyzoom_OFILM				0x1120
#define		gy2x4xb_OFILM				0x1121
#define		gylens_OFILM				0x1122
#define		gyta_OFILM				0x1123
#define		gytb_OFILM				0x1124
#define		gytc_OFILM				0x1125
#define		gytd_OFILM				0x1126
#define		gyte_OFILM				0x1127
#define		gylmt1H_OFILM				0x1128
#define		gylmt3HS0_OFILM			0x1129
#define		gylmt3HS1_OFILM			0x112A
#define		gylmt4HS0_OFILM			0x112B
#define		gylmt4HS1_OFILM			0x112C
#define		gylmt6L_OFILM				0x112D
#define		gylmt6H_OFILM				0x112E
#define		npylev3_OFILM				0x112F
#define		gyj1a_OFILM				0x1130
#define		gyj1b_OFILM				0x1131
#define		gyj1c_OFILM				0x1132
#define		gyj2a_OFILM				0x1133
#define		gyj2b_OFILM				0x1134
#define		gyj2c_OFILM				0x1135
#define		gyk1a_OFILM				0x1136
#define		gyk1b_OFILM				0x1137
#define		gyk1c_OFILM				0x1138
#define		gyk2a_OFILM				0x1139
#define		gyk2b_OFILM				0x113A
#define		gyk2c_OFILM				0x113B
#define		gyoa_OFILM				0x113C
#define		gyob_OFILM				0x113D
#define		gyoc_OFILM				0x113E
#define		npylev4_OFILM				0x113F
#define		MSABS2_OFILM				0x1140
#define		MSABS2AV_OFILM			0x1141
#define		MSPP2AV_OFILM				0x1142
#define		gyia_1_OFILM				0x1143
#define		gyib_1_OFILM				0x1144
#define		gyic_1_OFILM				0x1145
#define		gyia_a_OFILM				0x1146
#define		gyib_a_OFILM				0x1147
#define		gyic_a_OFILM				0x1148
#define		gyia_b_OFILM				0x1149
#define		gyib_b_OFILM				0x114A
#define		gyic_b_OFILM				0x114B
#define		gyia_c_OFILM				0x114C
#define		gyib_c_OFILM				0x114D
#define		gyic_c_OFILM				0x114E
#define		Stty12aM_OFILM			0x114F
#define		MSMAX2_OFILM				0x1150
#define		MSMAX2AV_OFILM			0x1151
#define		MSCT2AV_OFILM				0x1152
#define		gyla_1_OFILM				0x1153
#define		gylb_1_OFILM				0x1154
#define		gylc_1_OFILM				0x1155
#define		gyla_a_OFILM				0x1156
#define		gylb_a_OFILM				0x1157
#define		gylc_a_OFILM				0x1158
#define		gyla_b_OFILM				0x1159
#define		gylb_b_OFILM				0x115A
#define		gylc_b_OFILM				0x115B
#define		gyla_c_OFILM				0x115C
#define		gylb_c_OFILM				0x115D
#define		gylc_c_OFILM				0x115E
#define		Stty12aH_OFILM			0x115F
#define		MSMIN2_OFILM				0x1160
#define		MSMIN2AV_OFILM			0x1161
#define		MS2AV_OFILM				0x1162
#define		gygyro_1_OFILM			0x1163
#define		gygyro_1d_OFILM			0x1164
#define		gygyro_1u_OFILM			0x1165
#define		gygyro_a_OFILM			0x1166
#define		gygyro_2d_OFILM			0x1167
#define		gygyro_2u_OFILM			0x1168
#define		gygyro_b_OFILM			0x1169
#define		gygyro_3d_OFILM			0x116A
#define		gygyro_3u_OFILM			0x116B
#define		gygyro_c_OFILM			0x116C
#define		gygyro_4d_OFILM			0x116D
#define		gygyro_4u_OFILM			0x116E
#define		Stty12bM_OFILM			0x116F
#define		GGStp_OFILM				0x1170
#define		GGMin_OFILM				0x1171
#define		GGMax_OFILM				0x1172
#define		gygain_1_OFILM			0x1173
#define		gygain_1d_OFILM			0x1174
#define		gygain_1u_OFILM			0x1175
#define		gygain_a_OFILM			0x1176
#define		gygain_2d_OFILM			0x1177
#define		gygain_2u_OFILM			0x1178
#define		gygain_b_OFILM			0x1179
#define		gygain_3d_OFILM			0x117A
#define		gygain_3u_OFILM			0x117B
#define		gygain_c_OFILM			0x117C
#define		gygain_4d_OFILM			0x117D
#define		gygain_4u_OFILM			0x117E
#define		Stty12bH_OFILM			0x117F
#define		GGStp2_OFILM				0x1180
#define		GGMin2_OFILM				0x1181
#define		GGMax2_OFILM				0x1182
#define		gyistp_1_OFILM			0x1183
#define		gyistp_1d_OFILM			0x1184
#define		gyistp_1u_OFILM			0x1185
#define		gyistp_a_OFILM			0x1186
#define		gyistp_2d_OFILM			0x1187
#define		gyistp_2u_OFILM			0x1188
#define		gyistp_b_OFILM			0x1189
#define		gyistp_3d_OFILM			0x118A
#define		gyistp_3u_OFILM			0x118B
#define		gyistp_c_OFILM			0x118C
#define		gyistp_4d_OFILM			0x118D
#define		gyistp_4u_OFILM			0x118E
#define		Stty34aM_OFILM			0x118F
#define		vma_OFILM					0x1190
#define		vmb_OFILM					0x1191
#define		vmc_OFILM					0x1192
#define		gyistp_OFILM				0x1193
#define		gyadjmin_OFILM			0x1194
#define		gyadjmax_OFILM			0x1195
#define		gyadjdn_OFILM				0x1196
#define		gyadjup_OFILM				0x1197
#define		gyog3_OFILM				0x1198
#define		gyog5_OFILM				0x1199
#define		gyog7_OFILM				0x119A
#define		npylev8_OFILM				0x119B
#define		sylmtb1_OFILM				0x119C
#define		SttyaL_OFILM				0x119D
#define		SttybL_OFILM				0x119E
#define		Stty34aH_OFILM			0x119F
#define		sylmtb2_OFILM				0x11A0
#define		pymaa_OFILM				0x11A1
#define		pymab_OFILM				0x11A2
#define		pymac_OFILM				0x11A3
#define		pymba_OFILM				0x11A4
#define		pymbb_OFILM				0x11A5
#define		pymbc_OFILM				0x11A6
#define		gyma_OFILM				0x11A7
#define		gymb_OFILM				0x11A8
#define		gymc_OFILM				0x11A9
#define		gymg_OFILM				0x11AA
#define		gyleva_OFILM				0x11AB
#define		gylevb_OFILM				0x11AC
#define		gylevc_OFILM				0x11AD
#define		gylevlow_OFILM			0x11AE
#define		Stty34bM_OFILM			0x11AF
#define		syria_OFILM				0x11B0
#define		syrib_OFILM				0x11B1
#define		syric_OFILM				0x11B2
#define		syiny_OFILM				0x11B3
#define		syinx_OFILM				0x11B4
#define		syggf_OFILM				0x11B5
#define		syag_OFILM				0x11B6
#define		sypr_OFILM				0x11B7
#define		sygy_OFILM				0x11B8
#define		sygx_OFILM				0x11B9
#define		syiexp3_OFILM				0x11BA
#define		syiexp2_OFILM				0x11BB
#define		syiexp1_OFILM				0x11BC
#define		syiexp0_OFILM				0x11BD
#define		syiexp_OFILM				0x11BE
#define		Stty34bH_OFILM			0x11BF
#define		syda_OFILM				0x11C0
#define		sydb_OFILM				0x11C1
#define		sydc_OFILM				0x11C2
#define		syea_OFILM				0x11C3
#define		syeb_OFILM				0x11C4
#define		syec_OFILM				0x11C5
#define		syua_OFILM				0x11C6
#define		syub_OFILM				0x11C7
#define		syuc_OFILM				0x11C8
#define		syia_OFILM				0x11C9
#define		syib_OFILM				0x11CA
#define		syic_OFILM				0x11CB
#define		syja_OFILM				0x11CC
#define		syjb_OFILM				0x11CD
#define		syjc_OFILM				0x11CE
#define		npylev1_i_OFILM			0x11CF
#define		syfa_OFILM				0x11D0
#define		syfb_OFILM				0x11D1
#define		syfc_OFILM				0x11D2
#define		syg_OFILM					0x11D3
#define		syg2_OFILM				0x11D4
#define		sysin_OFILM				0x11D5
#define		syggf_tmp_OFILM			0x11D6
#define		sysa_OFILM				0x11D7
#define		sysb_OFILM				0x11D8
#define		sysc_OFILM				0x11D9
#define		syoa_OFILM				0x11DA
#define		syob_OFILM				0x11DB
#define		syoc_OFILM				0x11DC
#define		syod_OFILM				0x11DD
#define		syoe_OFILM				0x11DE
#define		npylev2_i_OFILM			0x11DF
#define		sypa_OFILM				0x11E0
#define		sypb_OFILM				0x11E1
#define		sypc_OFILM				0x11E2
#define		sypd_OFILM				0x11E3
#define		sype_OFILM				0x11E4
#define		syq_OFILM					0x11E5
#define		sylmta1_OFILM				0x11E6
#define		sylmta2_OFILM				0x11E7
#define		smyga_OFILM				0x11E8
#define		smygb_OFILM				0x11E9
#define		smya_OFILM				0x11EA
#define		smyb_OFILM				0x11EB
#define		syemglev_OFILM			0x11EC
#define		sysmtav_OFILM				0x11ED
#define		sysmtstp_OFILM			0x11EE
#define		npylev3_i_OFILM			0x11EF
#define		mes2aa_OFILM				0x11F0
#define		mes2ab_OFILM				0x11F1
#define		mes2ac_OFILM				0x11F2
#define		mes2ad_OFILM				0x11F3
#define		mes2ae_OFILM				0x11F4
#define		mes2ba_OFILM				0x11F5
#define		mes2bb_OFILM				0x11F6
#define		mes2bc_OFILM				0x11F7
#define		mes2bd_OFILM				0x11F8
#define		mes2be_OFILM				0x11F9
#define		syoexp3_OFILM				0x11FA
#define		syoexp2_OFILM				0x11FB
#define		syoexp1_OFILM				0x11FC
#define		syoexp0_OFILM				0x11FD
#define		syoexp_OFILM				0x11FE
#define		npylev4_i_OFILM			0x11FF
#define		afsin_OFILM				0x1200
#define		afing_OFILM				0x1201
#define		afstmg_OFILM				0x1202
#define		afag_OFILM				0x1203
#define		afda_OFILM				0x1204
#define		afdb_OFILM				0x1205
#define		afdc_OFILM				0x1206
#define		afea_OFILM				0x1207
#define		afeb_OFILM				0x1208
#define		afec_OFILM				0x1209
#define		afua_OFILM				0x120A
#define		afub_OFILM				0x120B
#define		afuc_OFILM				0x120C
#define		afia_OFILM				0x120D
#define		afib_OFILM				0x120E
#define		afic_OFILM				0x120F
#define		afja_OFILM				0x1210
#define		afjb_OFILM				0x1211
#define		afjc_OFILM				0x1212
#define		affa_OFILM				0x1213
#define		affb_OFILM				0x1214
#define		affc_OFILM				0x1215
#define		afg_OFILM					0x1216
#define		afg2_OFILM				0x1217
#define		afpa_OFILM				0x1218
#define		afpb_OFILM				0x1219
#define		afpc_OFILM				0x121A
#define		afpd_OFILM				0x121B
#define		afpe_OFILM				0x121C
#define		afstma_OFILM				0x121D
#define		afstmb_OFILM				0x121E
#define		afstmc_OFILM				0x121F
#define		aflmt_OFILM				0x1220
#define		aflmt2_OFILM				0x1221
#define		afssmv1_OFILM				0x1222
#define		afssmv2_OFILM				0x1223
#define		afsjlev_OFILM				0x1224
#define		afsjdif_OFILM				0x1225
#define		SttxHis_OFILM				0x1226
#define		tmpa_OFILM				0x1227
#define		af_cc_OFILM				0x1228
#define		a_df_OFILM				0x1229
#define		b_df_OFILM				0x122A
#define		c_df_OFILM				0x122B
#define		d_df_OFILM				0x122C
#define		e_df_OFILM				0x122D
#define		f_df_OFILM				0x122E
#define		pi_OFILM					0x122F
#define		msmean_OFILM				0x1230
#define		vmlevhis_OFILM			0x1231
#define		vmlev_OFILM				0x1232
#define		vmtl_OFILM				0x1233
#define		vmth_OFILM				0x1234
#define		st1mean_OFILM				0x1235
#define		st2mean_OFILM				0x1236
#define		st3mean_OFILM				0x1237
#define		st4mean_OFILM				0x1238
#define		dm1g_OFILM				0x1239
#define		dm2g_OFILM				0x123A
#define		dm3g_OFILM				0x123B
#define		dm4g_OFILM				0x123C
#define		zero_OFILM				0x123D
#define		com10_OFILM				0x123E
#define		cop10_OFILM				0x123F

//==============================================================================
//FILTER DELAY RAM
//==============================================================================
#define		SINXZ_OFILM				0x1400
#define		GX45Z_OFILM				0x1401
#define		GXINZ_OFILM				0x1402
#define		GXI1Z1_OFILM				0x1403
#define		GXI1Z2_OFILM				0x1404
#define		GXI2Z1_OFILM				0x1405
#define		GXI2Z2_OFILM				0x1406
#define		GXMZ1_OFILM				0x1407
#define		GXMZ2_OFILM				0x1408
#define		GXIZ_OFILM				0x1409
#define		GXXFZ_OFILM				0x140A
#define		GXADJZ_OFILM				0x140B
#define		GXGAINZ_OFILM				0x140C
#define		GXLEV1Z1_OFILM			0x140D
#define		GXLEV1Z2_OFILM			0x140E
#define		TMPX_OFILM				0x140F
#define		SXDOFFZ2_OFILM			0x1410
#define		GXH1Z1_OFILM				0x1411
#define		GXH1Z2_OFILM				0x1412
//#define							0x1413
#define		GXH2Z1_OFILM				0x1414
#define		GXH2Z2_OFILM				0x1415
#define		GXLEV2Z1_OFILM			0x1416
#define		GXH3Z1_OFILM				0x1417
#define		GXH3Z2_OFILM				0x1418
#define		GXL1Z1_OFILM				0x1419
#define		GXL1Z2_OFILM				0x141A
#define		GXL2Z1_OFILM				0x141B
#define		GXL2Z2_OFILM				0x141C
#define		GXL3Z_OFILM				0x141D
#define		GXLZ_OFILM				0x141E
#define		GXI3Z_OFILM				0x141F
#define		GXZOOMZ_OFILM				0x1420
#define		GXXBZ_OFILM				0x1421
#define		GXLENSZ_OFILM				0x1422
#define		GXLMT3Z_OFILM				0x1423
#define		GXTZ1_OFILM				0x1424
#define		GXTZ2_OFILM				0x1425
#define		GXTZ3_OFILM				0x1426
#define		GXTZ4_OFILM				0x1427
#define		GX2SXZ_OFILM				0x1428
#define		SXOVRZ_OFILM				0x1429
#define		PXAMZ_OFILM				0x142A
#define		PXMAZ1_OFILM				0x142B
#define		PXMAZ2_OFILM				0x142C
#define		PXBMZ_OFILM				0x142D
#define		PXMBZ1_OFILM				0x142E
#define		PXMBZ2_OFILM				0x142F
#define		DAXHLOtmp_OFILM			0x1430
#define		GXJ1Z1_OFILM				0x1431
#define		GXJ1Z2_OFILM				0x1432
#define		SXINZ1_OFILM				0x1433
#define		GXJ2Z1_OFILM				0x1434
#define		GXJ2Z2_OFILM				0x1435
#define		SXINZ2_OFILM				0x1436
#define		GXK1Z1_OFILM				0x1437
#define		GXK1Z2_OFILM				0x1438
#define		SXTRZ_OFILM				0x1439
#define		GXK2Z1_OFILM				0x143A
#define		GXK2Z2_OFILM				0x143B
#define		SXIEXPZ_OFILM				0x143C
#define		GXOZ1_OFILM				0x143D
#define		GXOZ2_OFILM				0x143E
#define		GXLEV2Z2_OFILM			0x143F
#define		AD0Z_OFILM				0x1440
#define		SXRIZ1_OFILM				0x1441
#define		SXRIZ2_OFILM				0x1442
#define		SXAGZ_OFILM				0x1443
#define		SXSMTZ_OFILM				0x1444
#define		MES1AZ1_OFILM				0x1445
#define		MES1AZ2_OFILM				0x1446
#define		MES1AZ3_OFILM				0x1447
#define		MES1AZ4_OFILM				0x1448
#define		SXTRZ1_OFILM				0x1449
#define		AD2Z_OFILM				0x144A
#define		MES1BZ1_OFILM				0x144B
#define		MES1BZ2_OFILM				0x144C
#define		MES1BZ3_OFILM				0x144D
#define		MES1BZ4_OFILM				0x144E
#define		AD4Z_OFILM				0x144F
#define		OFF0Z_OFILM				0x1450
#define		SXDZ1_OFILM				0x1451
#define		SXDZ2_OFILM				0x1452
#define		NPXDIFZ_OFILM				0x1453
#define		SXEZ1_OFILM				0x1454
#define		SXEZ2_OFILM				0x1455
#define		SX2HXZ2_OFILM				0x1456
#define		SXUZ1_OFILM				0x1457
#define		SXUZ2_OFILM				0x1458
#define		SXTRZ2_OFILM				0x1459
#define		OFF2Z_OFILM				0x145A
#define		SXIZ1_OFILM				0x145B
#define		SXIZ2_OFILM				0x145C
#define		SXJZ1_OFILM				0x145D
#define		SXJZ2_OFILM				0x145E
#define		OFF4Z_OFILM				0x145F
#define		AD0OFFZ_OFILM				0x1460
#define		SXOFFZ1_OFILM				0x1461
#define		SXOFFZ2_OFILM				0x1462
#define		SXFZ_OFILM				0x1463
#define		SXGZ_OFILM				0x1464
#define		NPXTMPZ_OFILM				0x1465
#define		SXG3Z_OFILM				0x1466
#define		SXSZ1_OFILM				0x1467
#define		SXSZ2_OFILM				0x1468
#define		SXTRZ3_OFILM				0x1469
#define		AD2OFFZ_OFILM				0x146A
#define		SXOZ1_OFILM				0x146B
#define		SXOZ2_OFILM				0x146C
#define		SXOZ3_OFILM				0x146D
#define		SXOZ4_OFILM				0x146E
#define		AD4OFFZ_OFILM				0x146F
#define		SXDOFFZ_OFILM				0x1470
#define		SXPZ1_OFILM				0x1471
#define		SXPZ2_OFILM				0x1472
#define		SXPZ3_OFILM				0x1473
#define		SXPZ4_OFILM				0x1474
#define		SXQZ_OFILM				0x1475
#define		SXOEXPZ_OFILM				0x1476
#define		SXLMT_OFILM				0x1477
#define		SX2HXZ_OFILM				0x1478
#define		DAXHLO_OFILM				0x1479
#define		DAXHLB_OFILM				0x147A
#define		TMPX2_OFILM				0x147B
#define		TMPX3_OFILM				0x147C
//#define							0x147D
//#define							0x147E
//#define							0x147F
#define		SINYZ_OFILM				0x1480
#define		GY45Z_OFILM				0x1481
#define		GYINZ_OFILM				0x1482
#define		GYI1Z1_OFILM				0x1483
#define		GYI1Z2_OFILM				0x1484
#define		GYI2Z1_OFILM				0x1485
#define		GYI2Z2_OFILM				0x1486
#define		GYMZ1_OFILM				0x1487
#define		GYMZ2_OFILM				0x1488
#define		GYIZ_OFILM				0x1489
#define		GYXFZ_OFILM				0x148A
#define		GYADJZ_OFILM				0x148B
#define		GYGAINZ_OFILM				0x148C
#define		GYLEV1Z1_OFILM			0x148D
#define		GYLEV1Z2_OFILM			0x148E
#define		TMPY_OFILM				0x148F
#define		SYDOFFZ2_OFILM			0x1490
#define		GYH1Z1_OFILM				0x1491
#define		GYH1Z2_OFILM				0x1492
//#define							0x1493
#define		GYH2Z1_OFILM				0x1494
#define		GYH2Z2_OFILM				0x1495
#define		GYLEV2Z1_OFILM			0x1496
#define		GYH3Z1_OFILM				0x1497
#define		GYH3Z2_OFILM				0x1498
#define		GYL1Z1_OFILM				0x1499
#define		GYL1Z2_OFILM				0x149A
#define		GYL2Z1_OFILM				0x149B
#define		GYL2Z2_OFILM				0x149C
#define		GYL3Z_OFILM				0x149D
#define		GYLZ_OFILM				0x149E
#define		GYI3Z_OFILM				0x149F
#define		GYZOOMZ_OFILM				0x14A0
#define		GYXBZ_OFILM				0x14A1
#define		GYLENSZ_OFILM				0x14A2
#define		GYLMT3Z_OFILM				0x14A3
#define		GYTZ1_OFILM				0x14A4
#define		GYTZ2_OFILM				0x14A5
#define		GYTZ3_OFILM				0x14A6
#define		GYTZ4_OFILM				0x14A7
#define		GY2SYZ_OFILM				0x14A8
#define		SYOVRZ_OFILM				0x14A9
#define		PYAMZ_OFILM				0x14AA
#define		PYMAZ1_OFILM				0x14AB
#define		PYMAZ2_OFILM				0x14AC
#define		PYBMZ_OFILM				0x14AD
#define		PYMBZ1_OFILM				0x14AE
#define		PYMBZ2_OFILM				0x14AF
#define		DAYHLOtmp_OFILM			0x14B0
#define		GYJ1Z1_OFILM				0x14B1
#define		GYJ1Z2_OFILM				0x14B2
#define		SYINZ1_OFILM				0x14B3
#define		GYJ2Z1_OFILM				0x14B4
#define		GYJ2Z2_OFILM				0x14B5
#define		SYINZ2_OFILM				0x14B6
#define		GYK1Z1_OFILM				0x14B7
#define		GYK1Z2_OFILM				0x14B8
#define		SYTRZ_OFILM				0x14B9
#define		GYK2Z1_OFILM				0x14BA
#define		GYK2Z2_OFILM				0x14BB
#define		SYIEXPZ_OFILM				0x14BC
#define		GYOZ1_OFILM				0x14BD
#define		GYOZ2_OFILM				0x14BE
#define		GYLEV2Z2_OFILM			0x14BF
#define		AD1Z_OFILM				0x14C0
#define		SYRIZ1_OFILM				0x14C1
#define		SYRIZ2_OFILM				0x14C2
#define		SYAGZ_OFILM				0x14C3
#define		SYSMTZ_OFILM				0x14C4
#define		MES2AZ1_OFILM				0x14C5
#define		MES2AZ2_OFILM				0x14C6
#define		MES2AZ3_OFILM				0x14C7
#define		MES2AZ4_OFILM				0x14C8
#define		SYTRZ1_OFILM				0x14C9
#define		AD3Z_OFILM				0x14CA
#define		MES2BZ1_OFILM				0x14CB
#define		MES2BZ2_OFILM				0x14CC
#define		MES2BZ3_OFILM				0x14CD
#define		MES2BZ4_OFILM				0x14CE
#define		AD5Z_OFILM				0x14CF
#define		OFF1Z_OFILM				0x14D0
#define		SYDZ1_OFILM				0x14D1
#define		SYDZ2_OFILM				0x14D2
#define		NPYDIFZ_OFILM				0x14D3
#define		SYEZ1_OFILM				0x14D4
#define		SYEZ2_OFILM				0x14D5
#define		SY2HYZ2_OFILM				0x14D6
#define		SYUZ1_OFILM				0x14D7
#define		SYUZ2_OFILM				0x14D8
#define		SYTRZ2_OFILM				0x14D9
#define		OFF3Z_OFILM				0x14DA
#define		SYIZ1_OFILM				0x14DB
#define		SYIZ2_OFILM				0x14DC
#define		SYJZ1_OFILM				0x14DD
#define		SYJZ2_OFILM				0x14DE
#define		OFF5Z_OFILM				0x14DF
#define		AD1OFFZ_OFILM				0x14E0
#define		SYOFFZ1_OFILM				0x14E1
#define		SYOFFZ2_OFILM				0x14E2
#define		SYFZ_OFILM				0x14E3
#define		SYGZ_OFILM				0x14E4
#define		NPYTMPZ_OFILM				0x14E5
#define		SYG3Z_OFILM				0x14E6
#define		SYSZ1_OFILM				0x14E7
#define		SYSZ2_OFILM				0x14E8
#define		SYTRZ3_OFILM				0x14E9
#define		AD3OFFZ_OFILM				0x14EA
#define		SYOZ1_OFILM				0x14EB
#define		SYOZ2_OFILM				0x14EC
#define		SYOZ3_OFILM				0x14ED
#define		SYOZ4_OFILM				0x14EE
#define		AD5OFFZ_OFILM				0x14EF
#define		SYDOFFZ_OFILM				0x14F0
#define		SYPZ1_OFILM				0x14F1
#define		SYPZ2_OFILM				0x14F2
#define		SYPZ3_OFILM				0x14F3
#define		SYPZ4_OFILM				0x14F4
#define		SYQZ_OFILM				0x14F5
#define		SYOEXPZ_OFILM				0x14F6
#define		SYLMT_OFILM				0x14F7
#define		SY2HYZ_OFILM				0x14F8
#define		DAYHLO_OFILM				0x14F9
#define		DAYHLB_OFILM				0x14FA
#define		TMPY2_OFILM				0x14FB
#define		TMPY3_OFILM				0x14FC
//#define							0x14FD
//#define							0x14FE
//#define							0x14FF
#define		AFSINZ_OFILM				0x1500
#define		AFDIFTMP_OFILM			0x1501
#define		AFINZ_OFILM				0x1502
#define		AFINZ2_OFILM				0x1503
#define		AFAGZ_OFILM				0x1504
#define		AFDZ1_OFILM				0x1505
#define		AFDZ2_OFILM				0x1506
#define		AFSTMGTSS_OFILM			0x1507
#define		AFEZ1_OFILM				0x1508
#define		AFEZ2_OFILM				0x1509
#define		OFSTAFZ_OFILM				0x150A
#define		AFUZ1_OFILM				0x150B
#define		AFUZ2_OFILM				0x150C
#define		AD4OFFZ2_OFILM			0x150D
#define		AFIZ1_OFILM				0x150E
#define		AFIZ2_OFILM				0x150F
#define		OFF6Z_OFILM				0x1510
#define		AFJZ1_OFILM				0x1511
#define		AFJZ2_OFILM				0x1512
#define		AFSTMTGT_OFILM			0x1513
#define		AFSTMSTP_OFILM			0x1514
#define		AFSTMTGTtmp_OFILM			0x1515
#define		AFFZ_OFILM				0x1516
#define		AFGZ_OFILM				0x1517
#define		AFG3Z_OFILM				0x1518
#define		AFPZ1_OFILM				0x1519
#define		AFPZ2_OFILM				0x151A
#define		AFPZ3_OFILM				0x151B
#define		AFPZ4_OFILM				0x151C
#define		AFLMTZ_OFILM				0x151D
#define		AF2PWM_OFILM				0x151E
#define		AFSTMZ2_OFILM				0x151F
#define		VMXYZ_OFILM				0x1520
#define		VMZ1_OFILM				0x1521
#define		VMZ2_OFILM				0x1522
//#define							0x1523
#define		OAFTHL_OFILM				0x1524
#define		PR_OFILM					0x1525
#define		AFRATO1_OFILM				0x1526
#define		ADRATO2_OFILM				0x1527
#define		AFRATO3_OFILM				0x1528
#define		DAZHLO_OFILM				0x1529
#define		DAZHLB_OFILM				0x152A
#define		AFL1Z_OFILM				0x152B
#define		AFL2Z_OFILM				0x152C
#define		AFDFZ_OFILM				0x152D
#define		pi_L1_OFILM				0x152E
#define		pi_L2_OFILM				0x152F

