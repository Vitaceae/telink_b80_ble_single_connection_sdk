/********************************************************************************************************
 * @file     main.c
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
extern my_fifo_t spp_rx_fifo;

/**
 * @brief   IRQ handler
 * @param   none.
 * @return  none.
 */
_attribute_ram_code_ void irq_handler(void)
{
	blc_sdk_irq_handler();
	
	if (reg_uart_status1 & FLD_UART_TX_DONE) {
		Tr_SetUartTxDone();
		uart_clr_tx_done();

	}
	if (dma_chn_irq_status_get() & FLD_DMA_CHN_UART_RX) {
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		u8* w = spp_rx_fifo.p + (spp_rx_fifo.wptr & (spp_rx_fifo.num - 1)) * spp_rx_fifo.size;
		if (w[0] != 0) {
			my_fifo_next(&spp_rx_fifo);
			u8* p = spp_rx_fifo.p + (spp_rx_fifo.wptr & (spp_rx_fifo.num - 1)) * spp_rx_fifo.size;
			reg_dma_uart_rx_addr = (u16) ((u32) p); //switch uart RX dma address
		}
	}
	if (uart_is_parity_error())//when stop bit error or parity error.
	{
		uart_clear_parity_error();
	}
}



/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_sec_noinline_
#endif

#if ZEROPLUS_CUST == 1
#define PIN_MY_ADC				B7P
#define MY_ADC_SAMPLE_NUM		8
volatile unsigned int adc_buf[MY_ADC_SAMPLE_NUM];

u32 my_adc_read(void)
{
	adc_reset_adc_module();
	u32 t0 = clock_time();
	u16 adc_sample[MY_ADC_SAMPLE_NUM] = {0};
	u32 adc_avg = 0;
	int i, j;

	for (i = 0; i < MY_ADC_SAMPLE_NUM; i++) {
		adc_buf[i] = 0;
	}
	while(!clock_time_exceed(t0, 25));

	adc_config_misc_channel_buf((u16 *)adc_buf, MY_ADC_SAMPLE_NUM << 2);
	dfifo_enable_dfifo2();
	for (i = 0; i < MY_ADC_SAMPLE_NUM; i++) {
		while (!adc_buf[i]) ;

		//14 bit resolution, BIT(13) is sign bit, 1 means negative voltage in differential_mode
		if (adc_buf[i] & BIT(13)) {
			adc_sample[i] = 0;
		} else {
			//BIT(12..0) is valid adc result
			adc_sample[i] = ((u16)adc_buf[i] & 0x1FFF);
		}

		//insert sort
		if (i) {
			if (adc_sample[i] < adc_sample[i - 1]) {
				u16 temp = adc_sample[i];
				adc_sample[i] = adc_sample[i - 1];
				for (j = i - 1; j >= 0 && adc_sample[j] > temp; j--) {
					adc_sample[j + 1] = adc_sample[j];
				}
				adc_sample[j + 1] = temp;
			}
		}
	}
	dfifo_disable_dfifo2();

	#if (MY_ADC_SAMPLE_NUM == 4)
		adc_avg = (adc_sample[1] + adc_sample[2]) / 2;
	#elif(MY_ADC_SAMPLE_NUM == 8)
		adc_avg = (adc_sample[2] + adc_sample[3] + adc_sample[4] + adc_sample[5]) / 4;
	#endif

	return adc_avg;
}

void my_adc_init(void)
{
	adc_init();
	adc_set_ain_chn_misc(PIN_MY_ADC, GND);
	adc_power_on_sar_adc(1);
}
#endif

int main(void)
{
	#if (BLE_MODULE_PM_ENABLE)
		blc_pm_select_internal_32k_crystal();
	#endif

	#if (BLE_MODULE_OTA_ENABLE && (FLASH_SIZE_OPTION == FLASH_SIZE_OPTION_128K))
		blc_ota_setFirmwareSizeAndBootAddress(48, MULTI_BOOT_ADDR_0x10000);
	#endif

	cpu_wakeup_init(EXTERNAL_XTAL_24M);

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_ble_1m_param_init();

	clock_init(SYS_CLK_TYPE);

	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again

	/* load customized freq_offset CAP value and TP value. */
	blc_app_loadCustomizedParameters();

	#if FIRMWARES_SIGNATURE_ENABLE
		blt_firmware_signature_check();
	#endif

	#if (PM_DEEPSLEEP_RETENTION_ENABLE)
		if( deepRetWakeUp ){
			user_init_deepRetn ();
		}
		else
	#endif
		{
			user_init_normal ();
		}

    irq_enable();

#if ZEROPLUS_CUST == 1
    my_adc_init();
	u32 adc_val_old = 0, adc_val;
	int loop_count = 0;

	printf("sys init\n");
#endif

	while (1) {
#if ZEROPLUS_CUST == 1
		loop_count++;
		if (loop_count >= 10000) {
			loop_count = 0;
			adc_val = my_adc_read();
			if (adc_val != adc_val_old) {
				adc_val_old = adc_val;
				printf("adc: %d\n", adc_val);
			} else {
				printf("no change\n");
			}
		}
#endif
		main_loop ();
	}
}

