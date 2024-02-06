/********************************************************************************************************
 * @file     feature_config.h
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

#ifndef FEATURE_CONFIG_H_
#define FEATURE_CONFIG_H_



/////////////////// TEST FEATURE SELECTION /////////////////////////////////
#define	TEST_FEATURE_BACKUP							0

//ble link layer test
#define TEST_POWER_ADV  							1
#define TEST_ADVERTISING_IN_CONN_SLAVE_ROLE  		2


#define TEST_SDATA_LENGTH_EXTENSION           		3
#define TEST_SLAVE_MD								4

#define TEST_USER_BLT_SOFT_TIMER                    5

#define	TEST_PHY_CONN								6

#define TEST_BLE_PHY								7	// BQB PHY_TEST demo
#define TEST_EMI									8		// emi test


#define TEST_GATT_SECURITY							9


#define TEST_USER_BLT_SOFT_UART                     10




#define TEST_SCANNING_IN_ADV_AND_CONN_SLAVE_ROLE 12


#define FEATURE_TEST_MODE							TEST_FEATURE_BACKUP



#endif /* FEATURE_CONFIG_H_ */
