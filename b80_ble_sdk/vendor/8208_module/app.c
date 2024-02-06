/********************************************************************************************************
 * @file     app.c
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
#include "app_att.h"
#include "app_buffer.h"
#include "spp.h"

#define 	ADV_IDLE_ENTER_DEEP_TIME			60  //60 s
#define 	CONN_IDLE_ENTER_DEEP_TIME			60  //60 s


#define MY_APP_ADV_CHANNEL			BLT_ENABLE_ADV_ALL
#define MY_ADV_INTERVAL_MIN			ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX			ADV_INTERVAL_35MS

#define MY_RF_POWER_INDEX			RF_POWER_P2p87dBm

#define MY_DIRECT_ADV_TMIE			10000000

#define	BLE_DEVICE_ADDRESS_TYPE 	BLE_DEVICE_ADDRESS_PUBLIC



u32	advertise_begin_tick;

own_addr_type_t app_own_address_type = OWN_ADDRESS_PUBLIC;

//module spp Tx / Rx fifo


 u8 		 	spp_rx_fifo_b[SPP_RXFIFO_SIZE * SPP_RXFIFO_NUM] = {0};
my_fifo_t	spp_rx_fifo = {
												SPP_RXFIFO_SIZE,
												SPP_RXFIFO_NUM,
												0,
												0,
												spp_rx_fifo_b,};

 u8 		 	spp_tx_fifo_b[SPP_TXFIFO_SIZE * SPP_TXFIFO_NUM] = {0};
my_fifo_t	spp_tx_fifo = {
												SPP_TXFIFO_SIZE,
												SPP_TXFIFO_NUM,
												0,
												0,
												spp_tx_fifo_b,};


/**
 * @brief	BLE Advertising data
 */
const u8	tbl_advData[] = {
	 0x05,  DT_COMPLETE_LOCAL_NAME, 				'e', 'M', 'o', 'd',
	 0x02,	 DT_FLAGS, 								0x05, 					// BLE limited discoverable mode and BR/EDR not supported
};

/**
 * @brief	BLE Scan Response Packet data
 */
const u8	tbl_scanRsp [] = {
	0x08,  DT_COMPLETE_LOCAL_NAME, 				'e', 'M', 'o', 'd', 'u', 'l', 'e',
};


u8 	ui_ota_is_working = 0;

#if (BLE_MODULE_OTA_ENABLE)

/**
 * @brief		callback function of ota start
 * @param[in]	none
 * @return      none
 */
void entry_ota_mode(void)
{
	bls_ota_setTimeout(15 * 1000 * 1000); //set OTA timeout  15 seconds

	#if(UI_LED_ENABLE)
		gpio_set_output_en(GPIO_LED_BLUE, 1);  //output enable
		gpio_write(GPIO_LED_BLUE, 1);  //LED on for indicate OTA mode
	#endif
}

/**
 * @brief		callback function of ota result
 * @param[in]	none
 * @return      none
 */
void show_ota_result(int result)
{

	#if(1 && UI_LED_ENABLE)
		gpio_set_output_en(GPIO_LED_BLUE, 1);

		if(result == OTA_SUCCESS){  //OTA success
			gpio_write(GPIO_LED_BLUE, 1);
			sleep_us(500000);
			gpio_write(GPIO_LED_BLUE, 0);
			sleep_us(500000);
			gpio_write(GPIO_LED_BLUE, 1);
			sleep_us(500000);
			gpio_write(GPIO_LED_BLUE, 0);
			sleep_us(500000);
		}
		else{  //OTA fail

			#if 0 //this is only for debug,  can not use this in application code
				irq_disable();
				WATCHDOG_DISABLE;


				while(1){
					gpio_write(GPIO_LED_BLUE, 1);
					sleep_us(200000);
					gpio_write(GPIO_LED_BLUE, 0);
					sleep_us(200000);
				}

			#endif

		}

		gpio_set_output_en(GPIO_LED_BLUE, 0);
	#endif
}
#endif


#define MAX_INTERVAL_VAL		16






u32 tick_wakeup;
int	mcu_uart_working;
int	module_uart_working;
int module_task_busy;


int	module_uart_data_flg;
u32 module_wakeup_module_tick;

#define UART_TX_BUSY			( (spp_tx_fifo.rptr != spp_tx_fifo.wptr) || Tr_isUartTxDone() )
#define UART_RX_BUSY			(spp_rx_fifo.rptr != spp_rx_fifo.wptr)

#if (BATT_CHECK_ENABLE)
u32	lowBattDet_tick   = 0;
#endif

/**
 * @brief		obtain uart working status
 * @param[in]	none
 * @return      0 for idle  else for busy
 */
int app_module_busy ()
{
	mcu_uart_working = gpio_read(GPIO_WAKEUP_MODULE);  //mcu use GPIO_WAKEUP_MODULE to indicate the UART data transmission or receiving state
	module_uart_working = UART_TX_BUSY || UART_RX_BUSY; //module checks to see if UART rx and tX are all processed
	module_task_busy = mcu_uart_working || module_uart_working;
	return module_task_busy;
}

/**
 * @brief		exit suspend mode
 * @param[in]	none
 * @return      none
 */
void app_suspend_exit ()
{
	GPIO_WAKEUP_MODULE_HIGH;  //module enter working state
	bls_pm_setSuspendMask(SUSPEND_DISABLE);
	tick_wakeup = clock_time () | 1;
	rf_set_power_level_index (MY_RF_POWER_INDEX);

}

/**
 * @brief		enter suspend mode
 * @param[in]	none
 * @return      0 - forbidden enter suspend mode
 *              1 - allow enter suspend mode
 */
int app_suspend_enter ()
{
	if (app_module_busy ())
	{
		app_suspend_exit ();
		return 0;
	}
	return 1;
}

/**
 * @brief      power management code for application
 * @param[in]  none
 * @return     none
 */
void app_power_management ()
{

#if (BLE_MODULE_INDICATE_DATA_TO_MCU)
	module_uart_working = UART_TX_BUSY || UART_RX_BUSY;

	//When module's UART data is sent, the GPIO_WAKEUP_MCU is lowered or suspended (depending on how the user is designed)
	if(module_uart_data_flg && !module_uart_working){
		module_uart_data_flg = 0;
		module_wakeup_module_tick = 0;
		GPIO_WAKEUP_MCU_LOW;
	}
#endif

	// pullup GPIO_WAKEUP_MODULE: exit from suspend
	// pulldown GPIO_WAKEUP_MODULE: enter suspend

#if (BLE_MODULE_PM_ENABLE)

	if (!app_module_busy() && !tick_wakeup)
	{
		#if (PM_DEEPSLEEP_RETENTION_ENABLE)
			bls_pm_setSuspendMask (SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);
		#else
			bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
		#endif

		bls_pm_setWakeupSource(PM_WAKEUP_PAD);  // GPIO_WAKEUP_MODULE needs to be wakened
	}

	if (tick_wakeup && clock_time_exceed (tick_wakeup, 500))
	{
		GPIO_WAKEUP_MODULE_LOW;
		tick_wakeup = 0;
	}

#endif
}


/**
 * @brief		user initialization when MCU power on or wake_up from deepSleep mode
 * @param[in]	none
 * @return      none
 */
void user_init_normal(void)
{
	/* random number generator must be initiated here( in the beginning of user_init_nromal).
	 * When deepSleep retention wakeUp, no need initialize again */
	random_generator_init();  //this is must


	/*****************************************************************************************
	 Note: battery check must do before any flash write/erase operation, cause flash write/erase
		   under a low or unstable power supply will lead to error flash operation

		   Some module initialization may involve flash write/erase, include: OTA initialization,
				SMP initialization, ..
				So these initialization must be done after  battery check
	*****************************************************************************************/
	#if (BATT_CHECK_ENABLE)  //battery check must do before OTA relative operation
		u8 battery_check_returnVaule = 0;
		if(analog_read(USED_DEEP_ANA_REG) & LOW_BATT_FLG){
			battery_check_returnVaule = app_battery_power_check(VBAT_ALRAM_THRES_MV + 200);  //2.2 V
		}
		else{
			battery_check_returnVaule = app_battery_power_check(VBAT_ALRAM_THRES_MV);  //2.0 V
		}
		if(battery_check_returnVaule){
			analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG)&(~LOW_BATT_FLG));  //clr
		}
		else{
			#if (UI_LED_ENABLE)  //led indicate
				for(int k=0;k<3;k++){
					gpio_write(GPIO_LED_BLUE, LED_ON_LEVAL);
					sleep_us(200000);
					gpio_write(GPIO_LED_BLUE, !LED_ON_LEVAL);
					sleep_us(200000);
				}
			#endif

			analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG)|LOW_BATT_FLG);  //mark
			GPIO_WAKEUP_FEATURE_LOW;

			cpu_set_gpio_wakeup (GPIO_WAKEUP_FEATURE, Level_High, 1);  //drive pin pad high wakeup deepsleep

			cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  //deepsleep
		}
	#endif


	//////////////////////////// BLE stack Initialization  Begin //////////////////////////////////
	u8 mac_public[6];
	u8 mac_random_static[6];
	blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);

	#if(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
		app_own_address_type = OWN_ADDRESS_PUBLIC;
	#elif(BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
		app_own_address_type = OWN_ADDRESS_RANDOM;
		blc_ll_setRandomAddr(mac_random_static);
	#endif

	//////////// LinkLayer Initialization  Begin /////////////////////////



	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(mac_public);				//mandatory
	blc_ll_initAdvertising_module();//adv module: mandatory for BLE slave,
	blc_ll_initSlaveRole_module();//slave module: mandatory for BLE slave,

	blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS, ACL_CONN_MAX_TX_OCTETS);
	blc_ll_initAclConnTxFifo(app_acl_txfifo, ACL_TX_FIFO_SIZE, ACL_TX_FIFO_NUM);
	blc_ll_initAclConnRxFifo(app_acl_rxfifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);

	u8 check_status = blc_controller_check_appBufferInitialization();
	if(check_status != BLE_SUCCESS){
		/* here user should set some log to know which application buffer incorrect */
		while(1);
	}
	//////////// LinkLayer Initialization  End /////////////////////////



	//////////// HCI Initialization  Begin /////////////////////////

	//////////// HCI Initialization  End /////////////////////////


	//////////// Controller Initialization  End /////////////////////////



	//////////// Host Initialization  Begin /////////////////////////
	/* GAP initialization must be done before any other host feature initialization !!! */
	blc_gap_peripheral_init();    //gap initialization

	/* GATT Initialization */
	my_gatt_init();

	/* L2CAP Initialization */
	blc_l2cap_register_handler(blc_l2cap_packet_receive);
	blc_l2cap_initRxDataBuffer(app_l2cap_rx_fifo, L2CAP_RX_BUFF_SIZE);


	/* SMP Initialization */
	/* SMP Initialization may involve flash write/erase(when one sector stores too much information,
	 *   is about to exceed the sector threshold, this sector must be erased, and all useful information
	 *   should re_stored) , so it must be done after battery check */
	#if (BLE_MODULE_SECURITY_ENABLE)
		bls_smp_configpairingSecurityInfoStorageAddr(FLASH_ADR_SMP_PAIRING);
		blc_smp_param_setBondingDeviceMaxNumber(4);  	//default is 4, can not bigger than this value

		blc_smp_peripheral_init();
		blc_smp_setSecurityLevel(Unauthenticated_Pairing_with_Encryption);

	#else
		blc_smp_setSecurityLevel(No_Security);
	#endif


	//////////// Host Initialization  End /////////////////////////

//////////////////////////// BLE stack Initialization  End //////////////////////////////////





//////////////////////////// User Configuration for BLE application ////////////////////////////
	u8 status = bls_ll_setAdvParam( MY_ADV_INTERVAL_MIN, MY_ADV_INTERVAL_MAX,
								 ADV_TYPE_CONNECTABLE_UNDIRECTED, app_own_address_type,
								 0,  NULL,
								 MY_APP_ADV_CHANNEL,
								 ADV_FP_NONE);

	if(status != BLE_SUCCESS) {  	while(1); }  //debug: adv setting err


	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
	bls_ll_setScanRspData( (u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));

	bls_ll_setAdvEnable(BLC_ADV_ENABLE);  //adv enable


	/* set rf power index, user must set it after every suspend wakeup, cause relative setting will be reset in suspend */
	rf_set_power_level_index (MY_RF_POWER_INDEX);


#if(TELINK_SPP_SERVICE_ENABLE)	
	////////////////// SPP initialization ///////////////////////////////////
	//note: dma addr must be set first before any other uart initialization!
	u8 *uart_rx_addr = (spp_rx_fifo_b + (spp_rx_fifo.wptr & (spp_rx_fifo.num-1)) * spp_rx_fifo.size);
	uart_recbuff_init( (unsigned char *)uart_rx_addr, spp_rx_fifo.size);

	uart_gpio_set(UART_TX_PIN, UART_RX_PIN);

	uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

	//baud rate: 115200
	uart_init_baudrate(UART_BAUD_RATE,CLOCK_SYS_CLOCK_HZ,PARITY_NONE, STOP_BIT_ONE);

	uart_dma_enable(1, 1); 	//uart data in hardware buffer moved by dma, so we need enable them first

	irq_enable_type(FLD_IRQ_DMA_EN);// uart_rx use dma_rx irq

	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX, 1);   	//uart Rx dma irq enable

	uart_mask_tx_done_irq_enable();

	uart_mask_error_irq_enable();// open uart_error_mask,when stop bit error or parity error,it will enter error_interrupt.
	irq_enable_type(FLD_IRQ_UART_EN);// uart_tx use uart_txdone irq

	Tr_SetUartTxDone();
	extern int rx_from_uart_cb (void);
	extern int tx_to_uart_cb (void);
	blc_register_hci_handler(rx_from_uart_cb, tx_to_uart_cb);				//customized uart handler


	extern int controller_event_handler(u32 h, u8 *para, int n);
	blc_hci_registerControllerEventHandler(controller_event_handler);		//register event callback
	bls_hci_mod_setEventMask_cmd(0xfffff);			//enable all 18 events,event list see ll.h


#endif	


#if (BLE_MODULE_PM_ENABLE)
	blc_ll_initPowerManagement_module();        //pm module:      	 optional

	#if (PM_DEEPSLEEP_RETENTION_ENABLE)
		blc_ll_initDeepsleepRetention_module();//Remove it if need save ramcode, and add DeepsleepRetentionEarlyWakeupTiming to 1ms
		bls_pm_setSuspendMask (SUSPEND_ADV | DEEPSLEEP_RETENTION_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_CONN);
		blc_pm_setDeepsleepRetentionThreshold(95, 95);
		blc_pm_setDeepsleepRetentionEarlyWakeupTiming(750);
	#else
		bls_pm_setSuspendMask (SUSPEND_ADV | SUSPEND_CONN);
	#endif


	//mcu can wake up module from suspend or deepsleep by pulling up GPIO_WAKEUP_MODULE
	cpu_set_gpio_wakeup (GPIO_WAKEUP_MODULE, Level_High, 1);  // pad high wakeup deepsleep

	GPIO_WAKEUP_MODULE_LOW;

	bls_pm_registerFuncBeforeSuspend( &app_suspend_enter );
#else
	bls_pm_setSuspendMask (SUSPEND_DISABLE);
#endif



#if (BLE_MODULE_OTA_ENABLE)
	// OTA init
	bls_ota_clearNewFwDataArea(); //must
	bls_ota_registerStartCmdCb(entry_ota_mode);
	bls_ota_registerResultIndicateCb(show_ota_result);
#endif

}


#if (PM_DEEPSLEEP_RETENTION_ENABLE)
/**
 * @brief		user initialization when MCU wake_up from deepSleep_retention mode
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ void user_init_deepRetn(void)
{
	//////////// LinkLayer Initialization  Begin /////////////////////////
	blc_ll_initBasicMCU();                      //mandatory

//////////////////////////// User Configuration for BLE application ////////////////////////////

	/* set rf power index, user must set it after every suspend wakeup, cause relative setting will be reset in suspend */
	rf_set_power_level_index (MY_RF_POWER_INDEX);

	blc_ll_recoverDeepRetention();


	////////////////// SPP initialization ///////////////////////////////////
	//note: dma addr must be set first before any other uart initialization!

	u8 *uart_rx_addr = (spp_rx_fifo_b + (spp_rx_fifo.wptr & (spp_rx_fifo.num-1)) * spp_rx_fifo.size);
	uart_recbuff_init( (unsigned char *)uart_rx_addr, spp_rx_fifo.size);


	uart_gpio_set(UART_TX_PIN, UART_RX_PIN);

	uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

	//baud rate: 115200
	uart_init_baudrate(115200,CLOCK_SYS_CLOCK_HZ,PARITY_NONE, STOP_BIT_ONE);

	uart_dma_enable(1, 1); 	//uart data in hardware buffer moved by dma, so we need enable them first

	irq_enable_type(FLD_IRQ_DMA_EN);// uart_rx use dma_rx irq

	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX, 1);   	//uart Rx dma irq enable

	uart_mask_tx_done_irq_enable();

	uart_mask_error_irq_enable();// open uart_error_mask,when stop bit error or parity error,it will enter error_interrupt.
	irq_enable_type(FLD_IRQ_UART_EN);// uart_tx use uart_txdone irq

	//mcu can wake up module from suspend or deepsleep by pulling up GPIO_WAKEUP_MODULE
	cpu_set_gpio_wakeup (GPIO_WAKEUP_MODULE, Level_High, 1);  // pad high wakeup deepsleep

	GPIO_WAKEUP_MODULE_LOW;
}
#endif


/**
 * @brief     BLE main loop
 * @param[in]  none.
 * @return     none.
 */
void main_loop (void)
{
	////////////////////////////////////// BLE entry /////////////////////////////////
	blc_sdk_main_loop();

	////////////////////////////////////// UI entry /////////////////////////////////

#if (BATT_CHECK_ENABLE)
	if(battery_get_detect_enable() && clock_time_exceed(lowBattDet_tick, 500000) ){
		lowBattDet_tick = clock_time();
		u8 battery_check_returnVaule;
		if(analog_read(USED_DEEP_ANA_REG) & LOW_BATT_FLG){
			battery_check_returnVaule=app_battery_power_check(VBAT_ALRAM_THRES_MV + 200);  //2.2 V
		}
		else{
			battery_check_returnVaule=app_battery_power_check(VBAT_ALRAM_THRES_MV);  //2.0 V
		}
		if(battery_check_returnVaule){
			analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG)&(~LOW_BATT_FLG));  //clr
		}
		else{
			#if (UI_LED_ENABLE)  //led indicate
				for(int k=0;k<3;k++){
					gpio_write(GPIO_LED_BLUE, LED_ON_LEVAL);
					sleep_us(200000);
					gpio_write(GPIO_LED_BLUE, !LED_ON_LEVAL);
					sleep_us(200000);
				}
			#endif

			analog_write(USED_DEEP_ANA_REG,  analog_read(USED_DEEP_ANA_REG)|LOW_BATT_FLG);  //mark
			GPIO_WAKEUP_FEATURE_LOW;

			cpu_set_gpio_wakeup (GPIO_WAKEUP_FEATURE, Level_High, 1);  //drive pin pad high wakeup deepsleep

			cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  //deepsleep
		}
	}
#endif

	/* module power management -----------------------------------------------*/
	app_power_management();

#if(TELINK_SPP_SERVICE_ENABLE)	
	spp_restart_proc();
#endif
}
