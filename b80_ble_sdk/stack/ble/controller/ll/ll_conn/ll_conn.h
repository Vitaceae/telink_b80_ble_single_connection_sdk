/********************************************************************************************************
 * @file     ll_conn.h
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

#ifndef LL_CONN_H_
#define LL_CONN_H_

/**
 * @brief	LE connection handle for slave role
 * controller now use 0x0040 in this SDK, never change, so user can use this value directly
 */
#define BLS_CONN_HANDLE 	0x0040

/**
 * @brief      for user to initialize LinkLayer TX FIFO.
 * 			   notice that: size*(number - 1)<= 0xFFF
 * @param[in]  pTxbuf - TX FIFO buffer address.
 * @param[in]  size - TX FIFO size,  must 16 Byte aligned
 * @param[in]  number - TX FIFO number, can only be 9, 17 or 33
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t 	blc_ll_initAclConnTxFifo(u8 *pTxbuf, int size, int number);


/**
 * @brief      for user to initialize LinkLayer RX FIFO.
 * @param[in]  pTxbuf - RX FIFO buffer address.
 * @param[in]  size - RX FIFO size
 * @param[in]  number - RX FIFO number, can only be 4, 8, 16 or 32
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t	blc_ll_initAclConnRxFifo(u8 *pRxbuf, int size, int number);



/**
 * @brief      for user to set connMaxRxOctets and connMaxTxOctets
 * @param[in]  maxRxOct - connMaxRxOctets, should be in range of 27 ~ 251
 * @param[in]  maxTxOct - connMaxTxOctets, should be in range of 27 ~ 251
 * @return     status, 0x00:  succeed
 * 					   other: failed
 */
ble_sts_t	blc_ll_setAclConnMaxOctetsNumber(u8 maxRxOct, u8 maxTxOct);

/**
 * @brief      for user to get connEffectiveMaxTxOctets
 * @param[in]  none
 * @return     connEffectiveMaxTxOctets
 */
u8 blc_ll_get_connEffectiveMaxTxOctets(void);


#endif /* LL_CONN_H_ */
