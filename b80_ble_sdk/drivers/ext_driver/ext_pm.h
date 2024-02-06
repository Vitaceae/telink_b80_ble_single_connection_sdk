/********************************************************************************************************
 * @file     ext_pm.h
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

#ifndef DRIVERS_8208_EXT_PM_H_
#define DRIVERS_8208_EXT_PM_H_

#ifndef PM_CUMULATIVE_ERROR_ELIMINATE_ENABLE
#define PM_CUMULATIVE_ERROR_ELIMINATE_ENABLE			    	0
#endif

#if (PM_CUMULATIVE_ERROR_ELIMINATE_ENABLE)

/**
 * @brief	early wakeup time
 */
typedef struct {
	unsigned short  suspend;	/*< suspend_early_wakeup_time_us >*/
	unsigned short  deep_ret;	/*< deep_ret_early_wakeup_time_us >*/
	unsigned short  deep;		/*< deep_early_wakeup_time_us >*/
	unsigned short  min;		/*< sleep_min_time_us >*/
	unsigned short	loopnum;	/*< the number of cycles for the crystal oscillator to be checked stably, 40us one time. >*/
}pm_early_wakeup_time_us_s;

extern volatile pm_early_wakeup_time_us_s g_pm_early_wakeup_time_us;

/**
 * @brief	hardware delay time
 */
typedef struct {
	unsigned short  deep_r_delay_cycle ;			/**< hardware delay time, deep_r_delay_us = (deep_r_delay_cycle+1) * 1/16k */
	unsigned short  suspend_ret_r_delay_cycle ;		/**< hardware delay time, suspend_ret_r_delay_us = (suspend_ret_r_delay_cycle+1) * 1/16k */
}pm_r_delay_cycle_s;

extern volatile pm_r_delay_cycle_s g_pm_r_delay_cycle;
typedef struct{
	unsigned int   tick_sysClk;
	unsigned int   tick_32k;
	unsigned int   recover_flag;
}pm_tim_recover_t;

extern _attribute_aligned_(4) pm_tim_recover_t			pm_timCalib;
#endif

extern unsigned int ota_program_bootAddr;
extern unsigned int ota_program_offset;
extern unsigned int ota_firmware_size_k;



/**
 * @brief		This function is used to enable the external crystal capacitor
 * @param[in]	en - enable the external crystal capacitor
 * @return      none
 */
static inline void blc_app_setExternalCrystalCapEnable(unsigned char  en)
{
	blt_miscParam.ext_cap_en = en;
}


/**
 * @brief      This function serves to set the working mode of MCU based on 32k rc,e.g. suspend mode, deepsleep mode, deepsleep with SRAM retention mode and shutdown mode.
 * 				  This saves size of RAM code but increases operating time and average power consumption.
 * @param[in]  sleep_mode - sleep mode type select.
 * @param[in]  wakeup_src - wake up source select.
 * @param[in]  wakeup_tick - the time of short sleep, which means MCU can sleep for less than 5 minutes.
 * @return     indicate whether the cpu is wake up successful.
 */
int  cpu_sleep_wakeup_32k_rc_text(SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, pm_wakeup_tick_type_e wakeup_tick_type, unsigned int  wakeup_tick);

/**
 * @brief      This function serves to set the working mode of MCU based on 32k crystal,e.g. suspend mode, deepsleep mode, deepsleep with SRAM retention mode and shutdown mode.
 *  			  This saves size of RAM code but increases operating time and average power consumption.
 * @param[in]  sleep_mode - sleep mode type select.
 * @param[in]  wakeup_src - wake up source select.
 * @param[in]  wakeup_tick - the time of short sleep, which means MCU can sleep for less than 5 minutes.
 * @return     indicate whether the cpu is wake up successful.
 */
int  cpu_sleep_wakeup_32k_xtal_text(SleepMode_TypeDef sleep_mode,  SleepWakeupSrc_TypeDef wakeup_src, pm_wakeup_tick_type_e wakeup_tick_type, unsigned int  wakeup_tick);


/**
 * @brief      This function serves to determine whether wake up source is internal 32k RC.This saves size of RAM code but increases operating time and average power consumption.
 * @param[in]  none.
 * @return     none.
 */
static inline void blc_pm_select_internal_32k_crystal_save_ram(void)
{
	cpu_sleep_wakeup_and_longsleep 	= cpu_sleep_wakeup_32k_rc_text;
	pm_tim_recover  	 			= pm_tim_recover_32k_rc;
	blt_miscParam.pm_enter_en 		= 1; // allow enter pm, 32k rc does not need to wait for 32k clk to be stable
}

/**
 * @brief      This function serves to determine whether wake up source is external 32k crystal.This saves size of RAM code but increases operating time and average power consumption.
 * @param[in]  none.
 * @return     none.
 */
static inline void blc_pm_select_external_32k_crystal_save_ram(void)
{
	cpu_sleep_wakeup_and_longsleep 	= cpu_sleep_wakeup_32k_xtal_text;
	pm_check_32k_clk_stable 		= check_32k_clk_stable;
	pm_tim_recover		 			= pm_tim_recover_32k_xtal;
	blt_miscParam.pad32k_en 		= 1; // set '1': 32k clk src use external 32k crystal
}

/*********************************************************/
//Remove when file merge to SDK //
typedef void (*pm_wakeup_init_t)(XTAL_TypeDef);
extern pm_wakeup_init_t pm_wakeup_init;

#endif

