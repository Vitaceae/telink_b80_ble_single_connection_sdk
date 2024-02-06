/********************************************************************************************************
 * @file     blt_common.h
 *
 * @brief    This is the header file for BLE SDK
 *
 * @author	 BLE GROUP
 * @date         12,2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/

#ifndef BLT_COMMON_H_
#define BLT_COMMON_H_


#include "drivers.h"
#include "vendor/common/user_config.h"


/**
 * @brief	128 K Flash MAC address and calibration data area
 */
#ifndef		CFG_ADR_MAC_128K_FLASH
#define		CFG_ADR_MAC_128K_FLASH								0x1F000
#endif

#ifndef		CFG_ADR_CALIBRATION_128K_FLASH
#define		CFG_ADR_CALIBRATION_128K_FLASH						0x1E000
#endif


/**
 * @brief	512 K Flash MAC address and calibration data area
 */
#ifndef		CFG_ADR_MAC_512K_FLASH
#define		CFG_ADR_MAC_512K_FLASH								0x7F000
#endif

#ifndef		CFG_ADR_CALIBRATION_512K_FLASH
#define		CFG_ADR_CALIBRATION_512K_FLASH						0x7E000
#endif






/**
 * @brief	Flash size option supported by this SDK
 */
#define		FLASH_SIZE_OPTION_128K			   					0x20000
#define		FLASH_SIZE_OPTION_512K			   					0x80000



#ifndef		FLASH_SIZE_OPTION
#define		FLASH_SIZE_OPTION		   							FLASH_SIZE_OPTION_128K
#endif





/**
 * @brief	Flash using area default Configuration, user can change some of them in app_config.h
 * 			CFG_ADR_MAC:  		  BLE MAC address stored in flash, can not change this value
 * 			CFG_ADR_CALIBRATION:  some calibration data stored in flash, can not change this value
 * 			FLASH_ADR_SMP_PAIRING:
 * 								  use 8K flash for SMP pairing information storage.
 * 									First 4K is for normal use, second 4K is a backup to guarantee SMP information never lose.
 * 									use API bls_smp_configpairingSecurityInfoStorageAddr(FLASH_ADR_SMP_PAIRING) to set the address.
 */
#if(FLASH_SIZE_OPTION == FLASH_SIZE_OPTION_128K)
	/* MAC and calibration data area */
	#define	CFG_ADR_MAC		   									CFG_ADR_MAC_128K_FLASH			//can not change this value
	#define	CFG_ADR_CALIBRATION		   							CFG_ADR_CALIBRATION_128K_FLASH	//can not change this value

/* SMP Pairing and key information area */
	#ifndef FLASH_ADR_SMP_PAIRING
	#define FLASH_ADR_SMP_PAIRING         						0x1C000
	#endif
#elif (FLASH_SIZE_OPTION == FLASH_SIZE_OPTION_512K)
	/* MAC and calibration data area */
	#define	CFG_ADR_MAC		   									CFG_ADR_MAC_512K_FLASH			//can not change this value
	#define	CFG_ADR_CALIBRATION		   							CFG_ADR_CALIBRATION_512K_FLASH	//can not change this value

	/* SMP pairing and key information area */
	#ifndef FLASH_ADR_SMP_PAIRING
	#define FLASH_ADR_SMP_PAIRING         						0x7C000
	#endif
#else
	#error "unsupported flash size !!!"
#endif











/** Calibration Information FLash Address Offset of  CFG_ADR_CALIBRATION_xx_FLASH ***/
#define		CALIB_OFFSET_CAP_INFO								0x00
#define		CALIB_OFFSET_TP_INFO								0x40
#define		CALIB_OFFSET_ADC_VREF								0xC0
#define		CALIB_OFFSET_FIRMWARE_SIGNKEY						0x180






/**
 * @brief		This function is used to load customized parameters from flash sector for application
 * @param[in]	none
 * @return      none
 */
static inline void blc_app_loadCustomizedParameters(void)
{
	unsigned char ana_8a = analog_read(0x8a);
	if(blt_miscParam.ext_cap_en) // 24M XTAL use external cap
	{
		ana_8a |= 0x80;  //BIT(7) set 1, turn off internal cap
		ana_8a &= 0xC0;  //BIT<6:0> clear, internal cap set 0
		analog_write(0x8a, ana_8a);
	}
	else{ //24M XTAL use internal cap
		unsigned char customed_cap;
		flash_read_page(CFG_ADR_CALIBRATION + CALIB_OFFSET_CAP_INFO, 1, &customed_cap);

		/* customize freq_offset adjust cap value, if not customized, default ana_8a is 0xd0 */
		if( customed_cap != 0xFF ){
			/* ana_8a<5:0> is cap value(0x00 - 0x3f) */
			unsigned char val = customed_cap & 0x3F;  //BIT<5:0> is valid, do not change BIT<7:6>
			ana_8a &= 0x40;  //BIT(7) clear, turn on internal cap; BIT<5:0> clear; BIT<6> not change
			ana_8a |= val;
			analog_write(0x8a, ana_8a);
		}
	}

}

/**
* @brief		This function is used to load customized ADC calibration from flash sector for application
* @param[in]	none
* @return      none
*/
static inline void blc_app_loadADCParameters(void)
{
	unsigned short vbat_calib_vref = 1175;
	signed char vbat_calib_vref_offset = 0;
	unsigned char adc_vref_calib_value_rd[4] = {0};
	flash_read_page(CFG_ADR_CALIBRATION+CALIB_OFFSET_ADC_VREF, 4, adc_vref_calib_value_rd);
	if(adc_vref_calib_value_rd[3] != 0xff)	//tmp_adc_cali[2] maybe 0xff
	{
		/******Method of calculating calibration Flash_vbat_Vref value: ********/
		/******Vref = [1175 +First_Byte-255+Second_Byte] mV =  [920 + First_Byte + Second_Byte] mV  ********/
		vbat_calib_vref = adc_vref_calib_value_rd[2] + 1000;
		vbat_calib_vref_offset = adc_vref_calib_value_rd[3] - 20;
		adc_set_vbat_calib_vref(vbat_calib_vref, vbat_calib_vref_offset);
	}

}


/**
 * @brief		This function is used to initialize the MAC address
 * @param[in]	flash_addr - flash address for MAC address
 * @param[in]	mac_public - public address
 * @param[in]	mac_random_static - random static MAC address
 * @return      none
 */
void blc_initMacAddress(int flash_addr, u8 *mac_public, u8 *mac_random_static);






#endif /* BLT_COMMON_H_ */
