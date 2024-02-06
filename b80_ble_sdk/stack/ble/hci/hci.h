/********************************************************************************************************
 * @file     hci.h
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



#define HCI_FLAG_EVENT_PHYTEST_2_WIRE_UART 	(1 << 23)
#define HCI_FLAG_EVENT_TLK_MODULE 			(1 << 24)
#define HCI_FLAG_EVENT_BT_STD 				(1 << 25)
#define HCI_FLAG_EVENT_STACK 				(1 << 26)
#define HCI_FLAG_ACL_BT_STD 				(1 << 27)




/**
 *  @brief  Definition for HCI packet type & HCI packet indicator
 */
typedef enum{
	HCI_TYPE_CMD 		= 0x01,
	HCI_TYPE_ACL_DATA	= 0x02,
	HCI_TYPE_SCO_DATA	= 0x03,
	HCI_TYPE_EVENT  	= 0x04,
	HCI_TYPE_ISO_DATA 	= 0x05,  //core_5.2
} hci_type_t;

/**
 *  @brief  Definition for HCI ACL Data packets Packet_Boundary_Flag
 */
typedef enum{
	HCI_FIRST_NAF_PACKET          =		0x00,	//LE Host to Controller
	HCI_CONTINUING_PACKET         =		0x01,	//LE Host to Controller / Controller to Host
	HCI_FIRST_AF_PACKET           =    	0x02,	//LE 					  Controller to Host
} acl_pb_flag_t;





// Controller event handler
/**
 * @brief	this function is used to register HCI Event handler Callback function
 */
typedef int (*hci_event_handler_t)(u32 h, u8 *para, int n);
extern hci_event_handler_t blc_hci_event_handler;



typedef int (*blc_hci_rx_handler_t)(void);
typedef int (*blc_hci_tx_handler_t)(void);



/**
 * @brief      this function is used to set HCI EVENT mask
 * @param[in]  evtMask  -  HCI EVENT mask
 * @return     0
 */
ble_sts_t	blc_hci_setEventMask_cmd(u32 evtMask);      //eventMask: BT/EDR


/**
 * @brief      this function is used to set HCI LE EVENT mask
 * @param[in]  evtMask  -  HCI LE EVENT mask(BIT<0-31>)
 * @return     0
 */
ble_sts_t	blc_hci_le_setEventMask_cmd(u32 evtMask);   //eventMask: LE event  0~31






/**
 * @brief      used to set telink defined event mask for BLE module only.
 * @param[in]  evtMask - event mask
 * @return     status, 0x00:  succeed
 * 			           other: failed
 */
ble_sts_t 	bls_hci_mod_setEventMask_cmd(u32 evtMask); //eventMask: module special



/**
 * @brief      this function is used to register HCI event handler callback function
 * @param[in]  handler - hci_event_handler_t
 * @return     none.
 */
void 		blc_hci_registerControllerEventHandler(hci_event_handler_t handler);



/**
 * @brief      this function is used to register HCI TX or RX handler callback function
 * @param[in]  *prx - blc_hci_rx_handler
 * @param[in]  *ptx - blc_hci_tx_handler
 * @return     none.
 */
void 		blc_register_hci_handler (void *prx, void *ptx);

/**
 * @brief      this function is used to send ACL data to HOST
 * @param[in]  handle - connect handle
 * @param[in]  *p - the pointer of l2cap data
 * @return     0
 */
int 		blc_hci_sendACLData2Host (u16 handle, u8 *p);


/**
 * @brief      this function is used to send data
 * @param[in]  h - HCI Event type
 * @param[in]  *para - data pointer of event
 * @param[in]  n - data length of event
 * @return     0,-1
 */
int 		blc_hci_send_data (u32 h, u8 *para, int n);


/**
 * @brief      this function is used to process HCI data
 * @param[in]  *p - the pointer of HCI data
 * @param[in]  n - the length of HCI data
 * @return     0
 */
int 		blc_hci_handler (u8 *p, int n);


/**
 * @brief      this function is used to process HCI data
 * @param[in]  rx_chn - the rx channel
 * @return     0
 */
ble_sts_t blc_phy_setReceiverTest (u8 rx_chn);


/**
 * @brief      this function is used to process HCI data
 * @param[in]  *pkt_num - the pointer of HCI data
 * @return     0
 */
ble_sts_t blc_phy_setPhyTestEnd (u8 *pkt_num);


/**
 * @brief      this function is used to process HCI data
 * @param[in]  tx_chn - the pointer of HCI data
 * @param[in]  length - the length of HCI data
 * @param[in]  pkt_type - the type of HCI data
 * @return     0
 */
ble_sts_t blc_phy_setTransmitterTest (u8 tx_chn, u8 length, u8 pkt_type);



