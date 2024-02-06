/********************************************************************************************************
 * @file     main.c
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
#include "stack/ble/ble.h"

#include "app.h"


#if (FEATURE_TEST_MODE == TEST_BLE_PHY)


/**
 * @brief   IRQ handler
 * @param   none.
 * @return  none.
 */
_attribute_ram_code_ void irq_handler(void)
{
	blc_sdk_irq_handler();

#if(FEATURE_TEST_MODE == TEST_BLE_PHY)
	app_phytest_irq_proc();
#endif
}





/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_sec_noinline_
#endif
int main(void)
{
	#if (BLE_APP_PM_ENABLE)
		blc_pm_select_internal_32k_crystal();
	#endif

	cpu_wakeup_init(EXTERNAL_XTAL_24M);

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_ble_1m_param_init();

	clock_init(SYS_CLK_TYPE);

	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

	/* load customized freq_offset CAP value and TP value. */
	blc_app_loadCustomizedParameters();

	#if (PM_DEEPSLEEP_RETENTION_ENABLE)
		if( deepRetWakeUp ){
			user_init_deepRetn ();
		}
		else
	#endif
		{
			user_init_normal ();
		}

    irq_enable();

	while (1) {
		main_loop ();
	}
}

#endif  //end of (FEATURE_TEST_MODE == xxx)
