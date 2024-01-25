/********************************************************************************************************
 * @file     app_config.h
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

#pragma once

#define ZEROPLUS_DEMO						1

///////////////////////// Feature Configuration////////////////////////////////////////////////
/**
 *  @brief  Feature select in BLE Module project
 */
#define FLASH_SIZE_OPTION							FLASH_SIZE_OPTION_512K //very important, user need confirm !!!
#define BLE_MODULE_SECURITY_ENABLE      					1
#define BLE_MODULE_OTA_ENABLE				1

/* firmware signature check */
#define FIRMWARES_SIGNATURE_ENABLE     		 		0

/* Power Management */
#define BLE_MODULE_PM_ENABLE							0
#define PM_DEEPSLEEP_RETENTION_ENABLE         0




/**
 *  @brief  UI Configuration
 */
#define UI_LED_ENABLE					1
#define TELINK_SPP_SERVICE_ENABLE			1
#define BLE_MODULE_INDICATE_DATA_TO_MCU		1
#define BATT_CHECK_ENABLE       			1   //enable or disable battery voltage detection



/**
 *  @brief  DEBUG  Configuration
 */
#define DEBUG_GPIO_ENABLE							0



///////////////////////// System Clock  Configuration /////////////////////////////////////////
#define CLOCK_SYS_CLOCK_HZ      					16000000


#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	#define SYS_CLK_TYPE  							SYS_CLK_16M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
	#define SYS_CLK_TYPE  							SYS_CLK_24M_Crystal
#elif(CLOCK_SYS_CLOCK_HZ == 32000000)
	#define SYS_CLK_TYPE  							SYS_CLK_32M_Crystal
#elif(CLOCK_SYS_CLOCK_HZ == 48000000)
	#define SYS_CLK_TYPE  							SYS_CLK_48M_Crystal
#else
	#error "unsupported system clock !"
#endif

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

/////////////////// DEEP SAVE FLG //////////////////////////////////
#define USED_DEEP_ANA_REG                   DEEP_ANA_REG0 //u8,can save 8 bit info when deep
#define	LOW_BATT_FLG					    BIT(0)

//////////////////////////// MODULE PM GPIO	/////////////////////////////////
#define GPIO_WAKEUP_MODULE					GPIO_PD5   //mcu wakeup module
#define	PD5_FUNC							AS_GPIO
#define PD5_INPUT_ENABLE					1
#define	PD5_OUTPUT_ENABLE					0
#define	PD5_DATA_OUT						0
#define GPIO_WAKEUP_MODULE_HIGH				gpio_setup_up_down_resistor(GPIO_WAKEUP_MODULE, PM_PIN_PULLUP_10K);
#define GPIO_WAKEUP_MODULE_LOW				gpio_setup_up_down_resistor(GPIO_WAKEUP_MODULE, PM_PIN_PULLDOWN_100K);

#define GPIO_WAKEUP_MCU						GPIO_PD6  //module wakeup mcu
#define	PD6_FUNC							AS_GPIO
#define PD6_INPUT_ENABLE					1
#define	PD6_OUTPUT_ENABLE					1
#define	PD6_DATA_OUT						0
#define GPIO_WAKEUP_MCU_HIGH				do{gpio_set_output_en(GPIO_WAKEUP_MCU, 1); gpio_write(GPIO_WAKEUP_MCU, 1);}while(0)
#define GPIO_WAKEUP_MCU_LOW					do{gpio_set_output_en(GPIO_WAKEUP_MCU, 1); gpio_write(GPIO_WAKEUP_MCU, 0);}while(0)
#define GPIO_WAKEUP_MCU_FLOAT				do{gpio_set_output_en(GPIO_WAKEUP_MCU, 0); gpio_write(GPIO_WAKEUP_MCU, 0);}while(0)

/**
 *  @brief  Battery_check Configuration
 */
#if (BATT_CHECK_ENABLE)
	#define ADC_INPUT_PCHN					VBAT    //corresponding  ADC_InputPchTypeDef in adc.h

	#define VBAT_ALRAM_THRES_MV				2000   // 2000 mV low battery alarm
	//////////////////////////// FEATURE PM GPIO	(EVK board) /////////////////////////////////
	#define GPIO_WAKEUP_FEATURE				GPIO_PB6   //mcu wakeup module
	#define	PB6_FUNC							AS_GPIO
	#define PB6_INPUT_ENABLE					1
	#define	PB6_OUTPUT_ENABLE					0
	#define	PB6_DATA_OUT						0
	#define GPIO_WAKEUP_FEATURE_HIGH				gpio_setup_up_down_resistor(GPIO_WAKEUP_FEATURE, PM_PIN_PULLUP_10K);
	#define GPIO_WAKEUP_FEATURE_LOW				gpio_setup_up_down_resistor(GPIO_WAKEUP_FEATURE, PM_PIN_PULLDOWN_100K);
#endif




#if(UI_LED_ENABLE)
		/**
		 *  @brief  Definition gpio for led
		 */
	#define	GPIO_LED_BLUE  	GPIO_PA4
	#define	GPIO_LED_GREEN	GPIO_PA5
	#define	GPIO_LED_WHITE	GPIO_PA6
	#define	GPIO_LED_RED		GPIO_PA7
	#define LED_ON_LEVAL 			1 		//gpio output high voltage to turn on led

	#define PA4_FUNC				AS_GPIO
	#define PA5_FUNC				AS_GPIO
	#define PA6_FUNC				AS_GPIO
	#define PA7_FUNC				AS_GPIO

	#define	PA4_OUTPUT_ENABLE		1
	#define	PA5_OUTPUT_ENABLE		1
	#define PA6_OUTPUT_ENABLE		1
	#define	PA7_OUTPUT_ENABLE		1
#endif



/////////////////////HCI UART variables///////////////////////////////////////
/*----------------------------------------------*
 *	SPP TX FIFO  = 2 Bytes LEN + n Bytes Data.	*
 *												*
 *	T_txdata_buf = 4 Bytes LEN + n Bytes Data.	*
 *												*
 *	SPP_TXFIFO_SIZE = 2 + n.					*
 *												*
 *	UART_DATA_LEN = n.							*
 *												*
 *	UART_DATA_LEN = SPP_TXFIFO_SIZE - 2.		*
 * ---------------------------------------------*/
#define SPP_RXFIFO_SIZE		64
#define SPP_RXFIFO_NUM		2

#define SPP_TXFIFO_SIZE		64
#define SPP_TXFIFO_NUM		8

#define UART_DATA_LEN    64      // data max 252
typedef struct{
    unsigned int len;        // data max 252
    unsigned char data[UART_DATA_LEN];
}uart_data_t;

#define UART_TX_PIN 		  GPIO_PB4
#define UART_RX_PIN 		  GPIO_PB5
#define UART_BAUD_RATE  115200
/**
 *  @brief  Definition for gpio debug
 */

#if(DEBUG_GPIO_ENABLE)
		#define PD0_FUNC				AS_GPIO //debug gpio chn1 : PD0
		#define PB1_FUNC				AS_GPIO //debug gpio chn2 : PB1
		#define PB2_FUNC				AS_GPIO //debug gpio chn3 : PB2
		#define PB3_FUNC                AS_GPIO //debug gpio chn4 : PB3
		#define PB4_FUNC				AS_GPIO //debug gpio chn5 : PB4
		#define PB5_FUNC				AS_GPIO //debug gpio chn6 : PB5
		#define PB6_FUNC				AS_GPIO //debug gpio chn7 : PB6
		#define PB7_FUNC				AS_GPIO //debug gpio chn0 : PB7

		#define PC0_FUNC				AS_GPIO //debug gpio chn8 : PC0
		#define PC1_FUNC                AS_GPIO //debug gpio chn9 : PC1
		#define PD2_FUNC				AS_GPIO //debug gpio chn10: PD2
		#define PD3_FUNC                AS_GPIO //debug gpio chn11: PD3
		#define PC4_FUNC				AS_GPIO //debug gpio chn12: PC4
		#define PC5_FUNC                AS_GPIO //debug gpio chn13: PC5
		#define PC6_FUNC				AS_GPIO //debug gpio chn14: PC6
		#define PC7_FUNC                AS_GPIO //debug gpio chn15: PC7

		#define GPIO_CHN0				GPIO_PD0
		#define GPIO_CHN1				GPIO_PB1
		#define GPIO_CHN2				GPIO_PB2
		#define GPIO_CHN3				GPIO_PB3
		#define GPIO_CHN4				GPIO_PB4
		#define GPIO_CHN5				GPIO_PB5
		#define GPIO_CHN6				GPIO_PB6
		#define GPIO_CHN7				GPIO_PB7

		#define GPIO_CHN8 				GPIO_PC0
		#define GPIO_CHN9 				GPIO_PC1
		#define GPIO_CHN10				GPIO_PD2
		#define GPIO_CHN11				GPIO_PD3
		#define GPIO_CHN12				GPIO_PC4
		#define GPIO_CHN13				GPIO_PC5
		#define GPIO_CHN14				GPIO_PC6
		#define GPIO_CHN15				GPIO_PC7

		#define PD0_OUTPUT_ENABLE					1
		#define PB1_OUTPUT_ENABLE					1
		#define PB2_OUTPUT_ENABLE					1
		#define PB3_OUTPUT_ENABLE					1
		#define PB4_OUTPUT_ENABLE					1
		#define PB5_OUTPUT_ENABLE					1
		#define PB6_OUTPUT_ENABLE					1
		#define PB7_OUTPUT_ENABLE					1

		#define PC0_OUTPUT_ENABLE					1
		#define PC1_OUTPUT_ENABLE					1
		#define PD2_OUTPUT_ENABLE					1
		#define PD3_OUTPUT_ENABLE					1
		#define PC4_OUTPUT_ENABLE					1
		#define PC5_OUTPUT_ENABLE					1
		#define PC6_OUTPUT_ENABLE					1
		#define PC7_OUTPUT_ENABLE					1
#endif  //end of DEBUG_GPIO_ENABLE








/////////////////// set default   ////////////////

#include "../common/default_config.h"
