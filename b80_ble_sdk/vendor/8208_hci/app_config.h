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

///////////////////////// Feature Configuration////////////////////////////////////////////////
/**
 *  @brief  Feature select in BLE Sample project
 */
#define FLASH_SIZE_OPTION							FLASH_SIZE_OPTION_512K //very important, user need confirm !!!
#define APP_SECURITY_ENABLE      					1
#define	BLE_OTA_SERVER_ENABLE						1

/* firmware signature check */
#define FIRMWARES_SIGNATURE_ENABLE     		 		0


/**
 *  @brief  UI Configuration
 */
#define UI_LED_ENABLE          	 					0

/* Power Management */
#define BLE_APP_PM_ENABLE							1


//////////////////////////// MODULE PM GPIO	/////////////////////////////////
#define GPIO_WAKEUP_MODULE					GPIO_PA0  //mcu wakeup module
#define	PA0_FUNC							AS_GPIO
#define PA0_INPUT_ENABLE					1
#define	PA0_OUTPUT_ENABLE					0
#define	PA0_DATA_OUT						0
#define GPIO_WAKEUP_MODULE_HIGH				gpio_setup_up_down_resistor(GPIO_WAKEUP_MODULE, PM_PIN_PULLUP_10K);
#define GPIO_WAKEUP_MODULE_LOW				gpio_setup_up_down_resistor(GPIO_WAKEUP_MODULE, PM_PIN_PULLDOWN_100K);


/**
 *  @brief HCI ACCESS OPTIONS
 */
#define HCI_USE_UART	1
#define HCI_USE_USB		0
#define HCI_ACCESS		HCI_USE_UART

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

enum {
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};


/////////////////////HCI UART variables///////////////////////////////////////
//module spp Tx / Rx fifo
/*----------------------------------------------*
 *	HCI TX FIFO  = 2 Bytes LEN + n Bytes Data.	*
 *												*
 *	T_txdata_buf = 4 Bytes LEN + n Bytes Data.	*
 *												*
 *	HCI_TXFIFO_SIZE = 2 + n.					*
 *												*
 *	UART_DATA_LEN = n.							*
 *												*
 *	UART_DATA_LEN = HCI_TXFIFO_SIZE - 2.		*
 * ---------------------------------------------*/
#define HCI_RXFIFO_SIZE			256
#define HCI_RXFIFO_NUM		    8

#define HCI_TXFIFO_SIZE			128
#define HCI_TXFIFO_NUM			8

#define UART_DATA_LEN    	(HCI_TXFIFO_SIZE - 2)      // data max 252
typedef struct {
	unsigned int len; // data max 252
	unsigned char data[UART_DATA_LEN];
} uart_data_t;

#define   UART_TX_PIN    		GPIO_PA4
#define   UART_RX_PIN    		GPIO_PA5
#define   UART_CTS_PIN  		GPIO_PA6
#define   UART_RTS_PIN   		GPIO_PA7
#define   UART__BAUDRATE        (115200)

#if(UI_LED_ENABLE)
	/**
	 *  @brief  Definition gpio for led
	 */
	#define	GPIO_LED_BLUE  			GPIO_PB3
	#define	GPIO_LED_GREEN			GPIO_PB4
	#define	GPIO_LED_WHITE			GPIO_PB5
	#define	GPIO_LED_RED			GPIO_PB6
	#define LED_ON_LEVAL 			1 		//gpio output high voltage to turn on led
	#define PB3_FUNC				AS_GPIO
	#define PB4_FUNC				AS_GPIO
	#define PB5_FUNC				AS_GPIO
	#define PB6_FUNC				AS_GPIO

	#define	PB3_OUTPUT_ENABLE		1
	#define	PB4_OUTPUT_ENABLE		1
	#define PB5_OUTPUT_ENABLE		1
	#define	PB6_OUTPUT_ENABLE		1
#endif

/**
 *  @brief  Definition for gpio debug
 */

#if(DEBUG_GPIO_ENABLE)
	#define PB0_FUNC				AS_GPIO //debug gpio chn1 : PB0
	#define PB1_FUNC				AS_GPIO //debug gpio chn2 : PB1
	#define PB2_FUNC				AS_GPIO //debug gpio chn3 : PB2
	#define PB3_FUNC                AS_GPIO //debug gpio chn4 : PB3
	#define PB4_FUNC				AS_GPIO //debug gpio chn5 : PB4
	#define PB5_FUNC				AS_GPIO //debug gpio chn6 : PB5
	#define PB6_FUNC				AS_GPIO //debug gpio chn7 : PB6
	#define PB7_FUNC				AS_GPIO //debug gpio chn0 : PB7
	#define PC0_FUNC				AS_GPIO //debug gpio chn8 : PC0
	#define PC1_FUNC                AS_GPIO //debug gpio chn9 : PC1
	#define PC2_FUNC				AS_GPIO //debug gpio chn10: PC2
	#define PC3_FUNC                AS_GPIO //debug gpio chn11: PC3
	#define PC4_FUNC				AS_GPIO //debug gpio chn12: PC4
	#define PC5_FUNC                AS_GPIO //debug gpio chn13: PC5
	#define PC6_FUNC				AS_GPIO //debug gpio chn14: PC6
	#define PC7_FUNC                AS_GPIO //debug gpio chn15: PC7
	#define GPIO_CHN0				GPIO_PB0
	#define GPIO_CHN1				GPIO_PB1
	#define GPIO_CHN2				GPIO_PB2
	#define GPIO_CHN3				GPIO_PB3
	#define GPIO_CHN4				GPIO_PB4
	#define GPIO_CHN5				GPIO_PB5
	#define GPIO_CHN6				GPIO_PB6
	#define GPIO_CHN7				GPIO_PB7

	#define GPIO_CHN8 				GPIO_PC0
	#define GPIO_CHN9 				GPIO_PC1
	#define GPIO_CHN10				GPIO_PC2
	#define GPIO_CHN11				GPIO_PC3
	#define GPIO_CHN12				GPIO_PC4
	#define GPIO_CHN13				GPIO_PC5
	#define GPIO_CHN14				GPIO_PC6
	#define GPIO_CHN15				GPIO_PC7

#endif  //end of DEBUG_GPIO_ENABLE
/////////////////// set default   ////////////////

#include "../common/default_config.h"
