/********************************************************************************************************
 * @file     ota.h
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

#ifndef BLE_LL_OTA_H_
#define BLE_LL_OTA_H_

#include "drivers.h"
#include "stack/ble/ble_common.h"

/**
 * @brief 	Legacy OTA command
 */
#define CMD_OTA_VERSION						0xFF00	//client -> server
#define CMD_OTA_START						0xFF01	//client -> server
#define CMD_OTA_END							0xFF02	//client -> server






/**
 * @brief	OTA result
 */
enum{
	OTA_SUCCESS 			= 0,			//success
	OTA_PACKET_LOSS,     					//lost one or more OTA PDU
	OTA_DATA_CRC_ERR,     					//packet PDU CRC err
	OTA_WRITE_FLASH_ERR,  					//write OTA data to flash ERR
 	OTA_DATA_UNCOMPLETE,  					//lost last one or more OTA PDU
 	OTA_TIMEOUT, 		  					//OTA flow total timeout
 	OTA_FW_CHECK_ERR,						//firmware CRC check error
 	OTA_STEP_ERR,

 	OTA_LOGIC_ERR,							//software logic error, please contact FAE of TeLink
};





/**
 * @brief	OTA start command callback declaration
 */
typedef void (*ota_startCb_t)(void);

/**
 * @brief	OTA version callback declaration
 */
typedef void (*ota_versionCb_t)(void);

/**
 * @brief		OTA result indicate callback declaration
 * @param[in]   result - OTA result
 */
typedef void (*ota_resIndicateCb_t)(int result);







/**
 * @brief      this function is used for user to initialize OTA server module.
 * 			   //attention: this API must called before any other OTA relative settings.
 * @param	   none
 * @return     none
 */
void blc_ota_initOtaServer_module(void);


/**
 * @brief      This function is used to set OTA new firmware storage address on Flash.
 * 			   note: this function must be called before "sys_init" or "cpu_wakeup_init".
 * @param[in]  firmware_size_k - set the firmware size. i.e. OTA erase flash size.note: unit is 1K(1024B)
 * @param[in]  new_fw_addr - new firmware storage address, 1.choose from multiple boot address
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t blc_ota_setFirmwareSizeAndBootAddress(int firmware_size_k, multi_boot_addr_e new_fw_addr);



/**
 * @brief      This function is used to register OTA start command callback.
 * 			   when local device receive OTA command  "CMD_OTA_START" and  "CMD_OTA_START_EXT"  form peer device,
 * 			   after checking all parameters are correct, local device will enter OTA update and trigger OTA start command callback.
 * @param[in]  cb - callback function
 * @return     none
 */
void blc_ota_registerOtaStartCmdCb(ota_startCb_t cb);


/**
 * @brief      This function is used to register OTA result indication callback
 * @param[in]  cb - callback function
 * @return     none
 */
void blc_ota_registerOtaResultIndicationCb(ota_resIndicateCb_t cb);

/**
 * @brief      This function is used to register OTA version command callback
 * 			   when local device receive OTA command  "CMD_OTA_VERSION", trigger this callback.
 * @param[in]  cb - callback function
 * @return     none
 */
void blc_ota_registerOtaFirmwareVersionReqCb(ota_versionCb_t cb);



/**
 * @brief      This function is used to set OTA whole process timeout value
 * 			   if not set, default value is 30 S.Counter will start from OTA_START
 * @param[in]  timeout_second - timeout value, unit: S, should in range of 4 ~ 1000
 * @return     Status - 0x00: command succeeded; 0x01-0xFF: command failed
 */
ble_sts_t blc_ota_setOtaProcessTimeout(int timeout_second);




/**
 * @brief      This function is used to write OTA data to flash
 * @param[in]  p - GATT data buffer pointer of write_req or write_cmd
 * @return
 */
int	otaWrite(void * p);









/* some API name compatible with other versions of SDK */

#define bls_ota_setFirmwareSizeAndOffset					blc_ota_setFirmwareSizeAndBootAddress

#define bls_ota_clearNewFwDataArea							blc_ota_initOtaServer_module

#define bls_ota_registerStartCmdCb							blc_ota_registerOtaStartCmdCb

#define bls_ota_registerResultIndicateCb					blc_ota_registerOtaResultIndicationCb

#define bls_ota_registerVersionReqCb						blc_ota_registerOtaFirmwareVersionReqCb


#define bls_ota_setTimeout(timeout_us)						blc_ota_setOtaProcessTimeout(timeout_us/1000000)

#endif /* BLE_LL_OTA_H_ */
