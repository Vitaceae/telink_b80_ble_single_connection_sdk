/********************************************************************************************************
 * @file     ll.h
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

#ifndef LL__H_
#define LL__H_






/**
 * @brief	BLE link layer state
 */
#define			BLS_LINK_STATE_IDLE								0
#define			BLS_LINK_STATE_ADV								BIT(0)
#define			BLS_LINK_STATE_SCAN								BIT(1)
#define			BLS_LINK_STATE_INIT								BIT(2)
#define 		BLS_LINK_STATE_CONN								BIT(3)



/**
 * @brief	Telink defined LinkLayer Event Callback Declaration
 */
typedef void (*blt_event_callback_t)(u8 e, u8 *p, int n);








/**
 * @brief	Telink defined LinkLayer Event Type
 */
#define BLT_EV_MAX_NUM 						21

#define BLT_EV_FLAG_ADV 					0
#define BLT_EV_FLAG_ADV_DURATION_TIMEOUT 	1
#define BLT_EV_FLAG_SCAN_RSP 				2
#define BLT_EV_FLAG_CONNECT 				3
#define BLT_EV_FLAG_TERMINATE 				4
#define BLT_EV_FLAG_PAIRING_BEGIN 			5
#define BLT_EV_FLAG_PAIRING_END 			6//success or fail(with fail reason)
#define BLT_EV_FLAG_ENCRYPTION_CONN_DONE 	7
#define BLT_EV_FLAG_DATA_LENGTH_EXCHANGE 	8
#define BLT_EV_FLAG_GPIO_EARLY_WAKEUP 		9
#define BLT_EV_FLAG_CHN_MAP_REQ 			10
#define BLT_EV_FLAG_CONN_PARA_REQ 			11
#define BLT_EV_FLAG_CHN_MAP_UPDATE 			12
#define BLT_EV_FLAG_CONN_PARA_UPDATE	 	13
#define BLT_EV_FLAG_SUSPEND_ENTER 			14
#define BLT_EV_FLAG_SUSPEND_EXIT 			15
#define BLT_EV_FLAG_RX_DATA_ABANDOM 		16
#define BLT_EV_FLAG_SMP_PINCODE_PROCESS 	17
#define BLT_EV_FLAG_SMP_KEY_MISSING 		18 //add for UTB2
#define BLT_EV_FLAG_PHY_UPDATE              19
#define BLT_EV_FLAG_REQ_INPUT_PIN           20







/**
 * @brief	irq_handler for BLE stack, process system tick interrupt and RF interrupt
 * @param	none
 * @return	none
 */
void 		blc_sdk_irq_handler(void);

/**
 * @brief   main_loop for BLE stack, process data and event
 * @param	none
 * @return	none
 */
int 		blc_sdk_main_loop (void);

/**
 * @brief   LinkLayer initialization after deepSleep retention wake_up
 * @param	none
 * @return	none
 */
void 		blc_ll_recoverDeepRetention(void);


/**
 * @brief      for user to initialize MCU
 * @param	   none
 * @return     none
 */
void 		blc_ll_initBasicMCU (void);



/**
 * @brief      for user to initialize link layer Standby state
 * @param[in]  *public_adr -  public address pointer
 * @return     none
 */
void 		blc_ll_initStandby_module (u8 *public_adr);




/**
 * @brief      this function is used to set the LE Random Device Address in the Controller
 * @param[in]  *randomAddr -  Random Device Address
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_setRandomAddr(u8 *randomAddr);


/**
 * @brief      this function is used to read MAC address
 * @param[in]  *addr -  The address where the read value(MAC address) prepare to write.
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_readBDAddr(u8 *addr);

/**
 * @brief      this function is used to get LE stack current state
 * @param[in]  none.
 * @return     BLE link layer state:
 * 					BLS_LINK_STATE_IDLE
 * 					BLS_LINK_STATE_ADV
 * 					BLS_LINK_STATE_SCAN
 * 					BLS_LINK_STATE_INIT
 * 					BLS_LINK_STATE_CONN
 */
u8 			blc_ll_getCurrentState(void);

/**
 * @brief      this function is used to get the most recent average RSSI
 * @param[in]  none.
 * @return     bltParam.ll_recentAvgRSSI
 */
u8 			blc_ll_getLatestAvgRSSI(void);


/**
 * @brief      this function is used to get TX FIFO Number of current state
 * @param[in]  none.
 * @return     total_fifo_num
 */
u8 blc_ll_getTxFifoNumber(void);

/**
 * @brief      this function is used to get current state of BRX
 * @param[in]  none.
 * @return     current state of BRX
 */
bool blc_ll_isBrxBusy (void);

/**
 * @brief		this function is used to register LinkLayer Event Callback function
 * @param[in]	e -
 * @param[in]	p -
 * @return		none
 */
void		bls_app_registerEventCallback (u8 e, blt_event_callback_t p);


/**
 * @brief      this function is used check if any controller buffer initialized by application incorrect.
 * 			   attention: this function must be called at the end of BLE LinkLayer Initialization.
 * @param	   none
 * @return     status, 0x00:  succeed, no buffer error
 * 					   other: buffer error code
 */
ble_sts_t	blc_controller_check_appBufferInitialization(void);


/**
 * @brief      this function is used to exchange data length.
 * @param[in]	opcode - LL Control PDU Opcode
 * @param[in]	maxTxOct - max TX octets of ACL_CONN
 * @return     status, 0x00:  succeed
 * 					   other: error code
 */
ble_sts_t 	blc_ll_exchangeDataLength (u8 opcode, u16 maxTxOct);


/**
 * @brief      this function is used to pend Controller event
 * @param[in]  none.
 * @return     blc_tlkEvent_pending
 */
bool		blc_ll_isControllerEventPending(void);









#endif /* LL__H_ */
