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
#include "app_buffer.h"



							u8 		 	hci_rx_fifo_b[HCI_RXFIFO_SIZE * HCI_RXFIFO_NUM] = {0};
my_fifo_t	hci_rx_fifo = {
												HCI_RXFIFO_SIZE,
												HCI_RXFIFO_NUM,
												0,
												0,
												hci_rx_fifo_b,};

							u8 		 	hci_tx_fifo_b[HCI_TXFIFO_SIZE * HCI_TXFIFO_NUM] = {0};
my_fifo_t	hci_tx_fifo = {
												HCI_TXFIFO_SIZE,
												HCI_TXFIFO_NUM,
												0,
												0,
												hci_tx_fifo_b,};
/////////////////////////////////////blc_register_hci_handler for spp////////////////////////////

u32 tick_wakeup;
int	mcu_uart_working;			//depends on the wakeup scheme, attention to the use
int	module_uart_working;
int module_task_busy;



#define UART_TX_BUSY			( (hci_tx_fifo.rptr != hci_tx_fifo.wptr) || uart_tx_is_busy() )
#define UART_RX_BUSY			(hci_rx_fifo.rptr != hci_rx_fifo.wptr)


/**
 * @brief      callback function of LinkLayer Event "BLT_EV_FLAG_SUSPEND_ENTER"
 * @param[in]  e - LinkLayer Event type
 * @param[in]  p - data pointer of event
 * @param[in]  n - data length of event
 * @return     none
 */
void app_set_kb_wakeup(u8 e, u8 *p, int n)
{
	#if (BLE_APP_PM_ENABLE)
		if( blc_ll_getCurrentState() == BLS_LINK_STATE_CONN
			&& ((u32)(bls_pm_getSystemWakeupTick() - clock_time())) > 80 *CLOCK_16M_SYS_TIMER_CLK_1MS ){  //suspend time > 30ms.add gpio wakeup
			bls_pm_setWakeupSource(PM_WAKEUP_PAD);  //gpio CORE wakeup suspend
		}
	#endif
}

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
	#if (BLE_APP_PM_ENABLE)

		if (!app_module_busy() && !tick_wakeup)
		{
			bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);

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
 * @brief		this function is used to process rx  data.
 * @param[in]	none
 * @return      0 is ok
 */
int  rx_from_uart_cb(void)//UART data send to Master,we will handler the data as CMD or DATA
{
	if (my_fifo_get(&hci_rx_fifo) == 0) {
		return 0;
	}

	u8* p = my_fifo_get(&hci_rx_fifo);
	u32 rx_len = p[0]; //usually <= 255 so 1 byte should be sufficient

	if (rx_len) {
		blc_hci_handler(&p[4], rx_len - 4);
		my_fifo_pop(&hci_rx_fifo);
	}

	return 0;
}

uart_data_t Tr_txdata_buf;
volatile u8 isUartTxDone = 1;
#define   Tr_clrUartTxDone()    (isUartTxDone = 0)
#define   Tr_SetUartTxDone()    (isUartTxDone = 1)
#define   Tr_isUartTxDone()     (!isUartTxDone)
/**
 * @brief		this function is used to process tx  data.
 * @param[in]	none
 * @return      0 is ok
 */
int tx_to_uart_cb(void) {
	if(hci_tx_fifo.wptr == hci_tx_fifo.rptr){
		return 0;//have no data
	}
	if (Tr_isUartTxDone()) {
		return 0;
	}
	u8 *p =  my_fifo_get(&hci_tx_fifo);
	memcpy(&Tr_txdata_buf.data, p + 2, p[0] | (p[1] << 8));

	Tr_txdata_buf.len = p[0] | (p[1] << 8);
	Tr_clrUartTxDone();
	uart_send_dma((unsigned char*) (&Tr_txdata_buf));
	my_fifo_pop(&hci_tx_fifo);
	return 1;
}

/**
 * @brief
 * @param[in]	none
 * @return      none
 */
void app_uart_irq_handler(void) {
	if (reg_uart_status1 & FLD_UART_TX_DONE) {
		Tr_SetUartTxDone();
		uart_clr_tx_done();
	}
	if (dma_chn_irq_status_get() & FLD_DMA_CHN_UART_RX) {
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		u8* w = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num - 1))
				* hci_rx_fifo.size;
		if (w[0] != 0) {
			my_fifo_next(&hci_rx_fifo);
			u8* p = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num - 1))
					* hci_rx_fifo.size;
			reg_dma_uart_rx_addr = (u16) ((u32) p); //switch uart RX dma address
		}
	}
	if (uart_is_parity_error())//when stop bit error or parity error.
	{
		uart_clear_parity_error();
	}
}
/**
 * @brief
 * @param[in]	none
 * @return      none
 */
void app_uart_init(void)
{
	//note: dma addr must be set first before any other uart initialization!
	uart_recbuff_init((unsigned char *) hci_rx_fifo_b, hci_rx_fifo.size);
	uart_gpio_set(UART_TX_PIN, UART_RX_PIN); // uart tx/rx pin set
	uart_reset(); //uart module power-on again.
	uart_init_baudrate(UART__BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
	uart_dma_enable(1, 1); //uart data in hardware buffer moved by dma, so we need enable them first
	irq_enable_type(FLD_IRQ_DMA_EN); // uart_rx use dma_rx irq
	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX, 1); //uart Rx dma irq enable
	uart_mask_tx_done_irq_enable();

	uart_mask_error_irq_enable();// open uart_error_mask,when stop bit error or parity error,it will enter error_interrupt.
	irq_enable_type(FLD_IRQ_UART_EN);// uart_tx use uart_txdone irq
	irq_enable();

	isUartTxDone = 1;
}

/**
 * @brief		user initialization when MCU power on or wake_up from deepSleep mode
 * @param[in]	none
 * @return      none
 */
void user_init_normal(void) {
	/* random number generator must be initiated here( in the beginning of user_init_nromal).
	 * When deepSleep retention wakeUp, no need initialize again */
	random_generator_init(); //this is must

	//////////////////////////// BLE stack Initialization  Begin //////////////////////////////////
	u8 mac_public[6];
	u8 mac_random_static[6];
	blc_initMacAddress(CFG_ADR_MAC, mac_public, mac_random_static);


	//////////// LinkLayer Initialization  Begin /////////////////////////
	blc_ll_initBasicMCU(); //mandatory
	blc_ll_initStandby_module(mac_public); //mandatory
	blc_ll_initAdvertising_module();//adv module: mandatory for BLE slave,
	blc_ll_initSlaveRole_module();//slave module: mandatory for BLE slave,

	blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS,
			ACL_CONN_MAX_TX_OCTETS);
	blc_ll_initAclConnTxFifo(app_acl_txfifo, ACL_TX_FIFO_SIZE, ACL_TX_FIFO_NUM);
	blc_ll_initAclConnRxFifo(app_acl_rxfifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);

	u8 check_status = blc_controller_check_appBufferInitialization();
	if (check_status != BLE_SUCCESS) {
		/* here user should set some log to know which application buffer incorrect */
		while (1);
	}

	//////////// LinkLayer Initialization  End /////////////////////////
	//////////// Controller Initialization  End /////////////////////////
	bls_ll_setAdvEnable(BLC_ADV_DISABLE);  //adv enable
	/* set rf power index, user must set it after every suspend wakeup, cause relative setting will be reset in suspend */
	rf_set_power_level_index (RF_POWER_P0p59dBm);
	//////////// HCI Initialization  Begin /////////////////////////
	app_uart_init();
	blc_register_hci_handler(rx_from_uart_cb, tx_to_uart_cb);//customized uart handler

	/* HCI Data && Event */
	blc_l2cap_register_handler (blc_hci_sendACLData2Host);  	//l2cap initialization
	blc_hci_registerControllerEventHandler(blc_hci_send_data); //controller hci event to host all processed in this func

	//bluetooth event
	blc_hci_setEventMask_cmd (HCI_EVT_MASK_DISCONNECTION_COMPLETE
			|HCI_EVT_MASK_ENCRYPTION_CHANGE);
	//bluetooth low energy(LE) event, all enable
	extern ble_sts_t 	blc_hci_le_setEventMask_cmd(u32 evtMask);
	blc_hci_le_setEventMask_cmd( 0xFFFFFFFF );
	//////////// HCI Initialization  End /////////////////////////

	//////////////////////////// BLE stack Initialization  End //////////////////////////////////

	///////////////////// Power Management initialization///////////////////
	#if (BLE_APP_PM_ENABLE)
		blc_ll_initPowerManagement_module();        //pm module:      	 optional


		bls_pm_setSuspendMask (SUSPEND_ADV | SUSPEND_CONN);

		//mcu can wake up module from suspend or deepsleep by pulling up GPIO_WAKEUP_MODULE
		cpu_set_gpio_wakeup (GPIO_WAKEUP_MODULE, Level_High, 1);  // pad high wakeup deepsleep

		GPIO_WAKEUP_MODULE_LOW;

		bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_ENTER, &app_set_kb_wakeup);
	#else
		bls_pm_setSuspendMask (SUSPEND_DISABLE);
	#endif
}

/**
 * @brief     BLE main loop
 * @param[in]  none.
 * @return     none.
 */
void main_loop(void) {

	////////////////////////////////////// BLE entry /////////////////////////////////
	blc_sdk_main_loop();

	app_power_management ();
	////////////////////////////////////// UI entry /////////////////////////////////
	#if (UI_LED_ENABLE)
		static u32 tickLoop = 1;
		if(tickLoop && clock_time_exceed(tickLoop, 500000)){
			tickLoop = clock_time();
			gpio_toggle(GPIO_LED_BLUE);
		}
	#endif

}

