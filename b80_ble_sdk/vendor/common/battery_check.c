/********************************************************************************************************
 * @file     battery_check.c
 *
 * @brief    This is the source file for BLE SDK
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
#include "tl_common.h"
#include "drivers.h"
#include <stack/ble/ble.h>
#include "../../common/config/user_config.h"
#include "battery_check.h"
#if (BATT_CHECK_ENABLE)

u8 		lowBattDet_enable = 1;
_attribute_data_reload_ u8      adc_hw_initialized = 0;   //note: can not be retention variable
 u16     batt_vol_mv;






#define ADC_SAMPLE_NUM		8



volatile unsigned int adc_dat_buf[ADC_SAMPLE_NUM];  //size must 16 byte aligned(16/32/64...)

/**
 * @brief		set lowBattery detect enable
 * @param[in]	en - lowBattDet_enable value
 * @return      none
 */
void battery_set_detect_enable (int en)
{
	lowBattDet_enable = en;

	if(!en){
		adc_hw_initialized = 0;   //need initialized again
	}

}

/**
 * @brief		get the value of lowBattDet_enable
 * @param[in]	none
 * @return      the value of lowBattDet_enable
 */
int battery_get_detect_enable (void)
{
	return lowBattDet_enable;
}

/**
 * @brief		This is battery check function
 * @param[in]	alram_vol_mv - input battery calue
 * @return      0 fail 1 success
 */
//_attribute_ram_code_ 
int app_battery_power_check(u16 alram_vol_mv)
{
	u16 temp;
	int i,j;

	//when MCU powered up or wakeup from deep/deep with retention, adc need be initialized
	if(!adc_hw_initialized){
		adc_hw_initialized = 1;
		adc_init();
		adc_vbat_channel_init();
		adc_power_on_sar_adc(1);
	}



	adc_reset_adc_module();
	u32 t0 = clock_time();


	u16 adc_sample[ADC_SAMPLE_NUM] = {0};
	u32 adc_result;

	for(i=0;i<ADC_SAMPLE_NUM;i++){   	//dfifo data clear
		adc_dat_buf[i] = 0;
	}
	while(!clock_time_exceed(t0, 25));  //wait at least 2 sample cycle(f = 96K, T = 10.4us)

	//dfifo setting will lose in suspend/deep, so we need config it every time
	adc_config_misc_channel_buf((u16 *)adc_dat_buf, ADC_SAMPLE_NUM<<2);  //size: ADC_SAMPLE_NUM*4
	dfifo_enable_dfifo2();





//////////////// get adc sample data and sort these data ////////////////
	for(i=0;i<ADC_SAMPLE_NUM;i++){
		while(!adc_dat_buf[i]);


		if(adc_dat_buf[i] & BIT(13)){  //14 bit resolution, BIT(13) is sign bit, 1 means negative voltage in differential_mode
			adc_sample[i] = 0;
		}
		else{
			adc_sample[i] = ((u16)adc_dat_buf[i] & 0x1FFF);  //BIT(12..0) is valid adc result
		}



		//insert sort
		if(i){
			if(adc_sample[i] < adc_sample[i-1]){
				temp = adc_sample[i];
				adc_sample[i] = adc_sample[i-1];
				for(j=i-1;j>=0 && adc_sample[j] > temp;j--){
					adc_sample[j+1] = adc_sample[j];
				}
				adc_sample[j+1] = temp;
			}
		}
	}
//////////////////////////////////////////////////////////////////////////////




	dfifo_disable_dfifo2();   //misc channel data dfifo disable





	///// get average value from raw data(abandon some small and big data ), then filter with history data //////
	#if (ADC_SAMPLE_NUM == 4)  	//use middle 2 data (index: 1,2)
		u32 adc_average = (adc_sample[1] + adc_sample[2])/2;
	#elif(ADC_SAMPLE_NUM == 8) 	//use middle 4 data (index: 2,3,4,5)
		u32 adc_average = (adc_sample[2] + adc_sample[3] + adc_sample[4] + adc_sample[5])/4;
	#endif




	#if 1
		adc_result = adc_average;
	#else  	//history data filter
		if(adc_first_flg){
			adc_result = adc_average;
			adc_first_flg = 0;
		}
		else{
			adc_result = ( (adc_result*3) + adc_average + 2 )>>2;  //filter
		}
	#endif


		//////////////// adc sample data convert to voltage(mv) ////////////////
		//                          (Vref, adc_pre_scale)   (BIT<12~0> valid data)
		//			 =  adc_result * Vref * adc_pre_scale / 0x2000 + offset
		//           =  adc_result * Vref*adc_pre_scale >>13 + offset
	extern unsigned char   adc_pre_scale;
	extern unsigned short g_adc_vref;
	extern signed char g_adc_vref_offset;
	batt_vol_mv  = ((adc_vbat_divider*adc_result*adc_pre_scale*g_adc_vref)>>13) + g_adc_vref_offset;

	if(batt_vol_mv < alram_vol_mv){
		return 0;
	}
	else{ // batt level > alarm level
		return 1;
	}
}

#endif
