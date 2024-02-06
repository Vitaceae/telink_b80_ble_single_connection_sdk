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


#include "feature_config.h"


#if(FEATURE_TEST_MODE == TEST_FEATURE_BACKUP)
	#include "feature_backup/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_ADVERTISING_IN_CONN_SLAVE_ROLE)
	#include "feature_adv_in_conn/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_SCANNING_IN_ADV_AND_CONN_SLAVE_ROLE)
	#include "feature_scan_in_adv_and_conn/app_config.h"
#elif (FEATURE_TEST_MODE == TEST_SDATA_LENGTH_EXTENSION)
	#include "feature_slave_dle/app_config.h"
#elif (FEATURE_TEST_MODE == TEST_SLAVE_MD)
	#include "feature_slave_md/app_config.h"
#elif (FEATURE_TEST_MODE ==TEST_USER_BLT_SOFT_TIMER)
	#include "feature_soft_timer/app_config.h"
#elif (FEATURE_TEST_MODE ==TEST_POWER_ADV)
	#include "feature_adv_power/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_PHY_CONN)
	#include "feature_phy_conn/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_BLE_PHY)
	#include "feature_phy_test/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_EMI)
	#include "feature_emi_test/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_USER_BLT_SOFT_UART)
	#include "feature_soft_uart/app_config.h"
#elif(FEATURE_TEST_MODE == TEST_GATT_SECURITY)
	#include "feature_gatt_security/app_config.h"
#endif


