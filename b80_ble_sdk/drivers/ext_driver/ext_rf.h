/********************************************************************************************************
 * @file     ext_rf.h
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

#ifndef DRIVERS_8208_EXT_RF_H_
#define DRIVERS_8208_EXT_RF_H_

#include "../driver.h"
#include "driver_ext.h"

enum{
	FLD_RF_R_CMD                 =	BIT_RNG(0,3),
	FLD_RF_R_STOP                =	0,
	FLD_RF_R_BTX                 =	1,
	FLD_RF_R_BRX                 =	2,
	FLD_RF_R_PTX                 =	3,
	FLD_RF_R_PRX                 =	4,
	FLD_RF_R_STX                 =	5,
	FLD_RF_R_SRX                 =	6,
	FLD_RF_R_STR                 =	7,
	FLD_RF_R_SRT                 =	8,
	FLD_RF_R_CMD_TRIG            =  BIT(7),
};


/**
 * @brief   This function serves to triggle accesscode in coded Phy mode.
 * @param   none.
 * @return  none.
 */
static inline void rf_trigle_codedPhy_accesscode(void)
{
	WRITE_REG8(0x405, REG_ADDR8(0x405) | BIT(7)); //reg_rf_acc_len |= FLD_RF_LR_ACC_TRIG;
}


typedef void (*rf_1m_init_t)(void);
extern rf_1m_init_t rf_ble_1m_param_init; ///This function serves to set RF  ble 1m Register initialization.

typedef enum{
	FSM_BTX 	= 0x81,
	FSM_BRX 	= 0x82,
	FSM_STX 	= 0x85,
	FSM_SRX 	= 0x86,
	FSM_TX2RX	= 0x87,
	FSM_RX2TX	= 0x88,
}fsm_mode_e;

/**
 * @brief     	This function serves to RF trigger RF state machine.
 * @param[in] 	mode  - FSM mode.
 * @param[in] 	tx_addr  - DMA TX buffer, if not TX, must be "NULL"
 * @param[in] 	tick  - FAM Trigger tick.
 * @return	   	none.
 */
void rf_start_fsm(fsm_mode_e mode, void* tx_addr, unsigned int tick);

/**
 * @brief     	This function serves to RF trigger RF state machine.
 * @param[in] 	WakeupSrc  - wakeup source.
 * @param[in] 	rf_mask  - rf irq mask
 * @param[in] 	tick  - system timer irq tick
 * @return	   	wakeup source.
 */
unsigned int cpu_stall_WakeUp_By_RF_SystemTick(int WakeupSrc, unsigned short rf_mask, unsigned int tick);

#define		rf_receiving_flag				is_rf_receiving_pkt ///static inline
#define		rf_tx_settle_adjust				tx_settle_adjust    ///static inline

#define 	RF_TX_PAKET_DMA_LEN(len)		(len)

#define     RF_BLE_RF_PAYLOAD_LENGTH_OK		RF_BLE_PACKET_LENGTH_OK
#define     RF_BLE_RF_PACKET_CRC_OK			RF_BLE_PACKET_CRC_OK
#define	RF_BLE_PACKET_VALIDITY_CHECK(p)					(RF_BLE_RF_PAYLOAD_LENGTH_OK(p) && RF_BLE_RF_PACKET_CRC_OK(p))

#define 	reg_rf_ll_cmd_schedule 			REG_ADDR32(0xf18)
#define 	reg_rf_ll_cmd					REG_ADDR8(0xf00)

#define		STOP_RF_STATE_MACHINE	(REG_ADDR8(0xf00) = 0x80)// stop FMS

#endif

