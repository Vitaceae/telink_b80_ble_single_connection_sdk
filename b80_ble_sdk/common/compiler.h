/********************************************************************************************************
 * @file	compiler.h
 *
 * @brief	This is the header file for B80
 *
 * @author	Driver Group
 * @date	2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
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
 *
 *******************************************************************************************************/
#pragma once
#include "driver_func_cfg.h"
#include "../../common/config/user_config.h"

#ifndef NULL
#define NULL 	0
#endif

#define _attribute_packed_					__attribute__((packed))
#define _attribute_aligned_(s)				__attribute__((aligned(s)))
#define _attribute_session_(s)				__attribute__((section(s)))
#define _attribute_ram_code_sec_      		__attribute__((section(".ram_code")))
#define _attribute_ram_code_sec_noinline_	__attribute__((section(".ram_code"))) __attribute__((noinline))
#define _attribute_text_sec_   				__attribute__((section(".text")))
#define _attribute_custom_code_  			_attribute_session_(".custom") volatile
#define _attribute_no_inline_   			__attribute__((noinline))
#define _inline_ 							inline

#ifndef	BLC_PM_DEEP_RETENTION_MODE_EN
#define	BLC_PM_DEEP_RETENTION_MODE_EN		0
#endif


#define _attribute_data_retention_
#define _attribute_data_reload_         __attribute__((section(".data_reload")))
