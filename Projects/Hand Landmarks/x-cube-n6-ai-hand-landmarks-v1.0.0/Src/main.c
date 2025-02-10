	 /**
	 ******************************************************************************
	 * @file    main.c
	 * @author  GPM Application Team
	 *
	 ******************************************************************************
	 * @attention
	 *
	 * Copyright (c) 2023 STMicroelectronics.
	 * All rights reserved.
	 *
	 * This software is licensed under terms that can be found in the LICENSE file
	 * in the root directory of this software component.
	 * If no LICENSE file comes with this software, it is provided AS-IS.
	 *
	 ******************************************************************************
	 */

	#include <assert.h>

	#include "app.h"
	#include "app_config.h"
	#include "app_fuseprogramming.h"
	#include "main.h"
	#include "npu_cache.h"
	#include "stm32n6570_discovery.h"
	#include "stm32n6570_discovery_bus.h"
	#include "stm32n6570_discovery_lcd.h"
	#include "stm32n6570_discovery_xspi.h"
	#include <stdio.h>
	#include "stm32n6xx_hal_rif.h"
	#include "tx_api.h"
	#include "tx_initialize.h"

	//iotc start
	#include "da16k_comm.h"
	UART_HandleTypeDef huart2;
	static TX_THREAD iotc_thread;
	static uint8_t iotc_thread_stack[4096];
	extern char iot_classification[32]; // Stores the detected hand gesture
	//iotc stop

	UART_HandleTypeDef huart1;

	static TX_THREAD main_thread;
	static uint8_t main_thread_stack[4096];

	static void SystemClock_Config(void);
	static void NPURam_enable();
	static void NPUCache_config();
	static void Security_Config();
	static void IAC_Config();
	static void CONSOLE_Config(void);
	static int main_threadx(void);
	static void main_thread_fct(ULONG arg);

	//iotc start
	/* CPU Load */
	extern float iot_cpu_percentage;

	/* Neural Network Performance */
	extern float iot_nn_fps;
	extern float iot_nn_period_ms;

	/* Inference Execution Times */
	extern float iot_pd_ms;  // Palm Detector execution time (ms)
	extern float iot_hl_ms;  // Hand Landmark execution time (ms)
	extern float iot_disp_ms; // Display update time (ms)

	/* Hand Detection */
	extern int iot_pd_hand_nb;   // Number of hands detected
	extern float iot_pd_max_prob;  // Maximum confidence of detected hands
	extern int iot_hand_valid;  // 1 = Hand detected, 0 = No hand detected

	extern float iot_hand_x_center, iot_hand_y_center;
	extern float iot_hand_width, iot_hand_height;
	extern float iot_hand_rotation;

	/* Hand Landmark Variables */
	extern float iot_wrist_x, iot_wrist_y;

	/* Thumb */
	extern float iot_thumb_cmc_x, iot_thumb_cmc_y;
	extern float iot_thumb_mcp_x, iot_thumb_mcp_y;
	extern float iot_thumb_ip_x, iot_thumb_ip_y;
	extern float iot_thumb_tip_x, iot_thumb_tip_y;

	/* Index Finger */
	extern float iot_index_mcp_x, iot_index_mcp_y;
	extern float iot_index_pip_x, iot_index_pip_y;
	extern float iot_index_dip_x, iot_index_dip_y;
	extern float iot_index_tip_x, iot_index_tip_y;

	/* Middle Finger */
	extern float iot_middle_mcp_x, iot_middle_mcp_y;
	extern float iot_middle_pip_x, iot_middle_pip_y;
	extern float iot_middle_dip_x, iot_middle_dip_y;
	extern float iot_middle_tip_x, iot_middle_tip_y;

	/* Ring Finger */
	extern float iot_ring_mcp_x, iot_ring_mcp_y;
	extern float iot_ring_pip_x, iot_ring_pip_y;
	extern float iot_ring_dip_x, iot_ring_dip_y;
	extern float iot_ring_tip_x, iot_ring_tip_y;

	/* Pinky Finger */
	extern float iot_pinky_mcp_x, iot_pinky_mcp_y;
	extern float iot_pinky_pip_x, iot_pinky_pip_y;
	extern float iot_pinky_dip_x, iot_pinky_dip_y;
	extern float iot_pinky_tip_x, iot_pinky_tip_y;

	#define TX_BUFFER_SIZE  512
	#define RX_BUFFER_SIZE  256
	static char command[TX_BUFFER_SIZE] = { 0 };
	static char response[RX_BUFFER_SIZE] = { 0 };
	extern da16k_err_t da16k_at_send_formatted_raw_no_crlf(const char *format, ...);
	extern float iot_cpu_percentage;

	static void MX_USART2_UART_Init(void)
	{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  __HAL_RCC_USART2_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
		/**USART2 GPIO Configuration
		PD5     ------> USART2_TX  D1
		PF6     ------> USART2_RX  D0
		*/
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /* USART2 interrupt Init */
	  //HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	  //HAL_NVIC_EnableIRQ(USART2_IRQn);

	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  //huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  //huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
		  while (1);
	  }
	}

	static char* send_at_command(char *command, unsigned long timeout_ms)
	{
	  int i = 0;
	  int command_size = strlen(command);
	  if (command_size > 255) {
		  printf("AT COMMAND is over size!\r\n");
		  return response;
	  }

	  HAL_StatusTypeDef USART_STATUS = HAL_OK;

	  //flush UART
	  while (HAL_TIMEOUT != HAL_UART_Receive(&huart2, (uint8_t* )&response[0], TX_BUFFER_SIZE, 50)) {
		  //do nothing
	  }
	  memset(response, 0, RX_BUFFER_SIZE);

	#if 1
	  printf("command sent: %s\n", command);
	#endif

	  if (HAL_UART_Transmit(&huart2, (uint8_t*)command, command_size, timeout_ms) != HAL_OK) {
	      printf("UART transmission error!\n");
	      return NULL;
	  }

	  do {
	      if (i >= RX_BUFFER_SIZE - 1) {
	          response[i] = '\0';  // Ensure null termination
	          break;
	      }
	      USART_STATUS = HAL_UART_Receive(&huart2, (uint8_t*)&response[i], 1, timeout_ms);
	      i++;
	  } while ((response[i - 1] != '\n') && (USART_STATUS != HAL_TIMEOUT));


	  if(USART_STATUS == HAL_TIMEOUT)
	  {
		memset  (response, 0, RX_BUFFER_SIZE);
		snprintf(response, TX_BUFFER_SIZE, "ERROR\r\n");
	  }

	#if 0
	  //printf("response received is %s\n", response);
	  // printf("len is %d\n", i);

	  printf("response is: \n");
	  for (int j = 0; j < i ; j++) {
		  printf("%d\n", response[j]);
	  }
	#endif
	  return response;
	}

	//extern int32_t nb_detect;
	//extern float32_t x_center;
	//extern float32_t y_center;

	int detect_hand_gesture() {
	    if (iot_hand_valid) { // Ensure a hand is detected
	        // Use function-local variables instead of modifying global ones
	        float wrist_baseline_y = iot_wrist_y;
	        float norm_index_tip_y = iot_index_tip_y - wrist_baseline_y;
	        float norm_middle_tip_y = iot_middle_tip_y - wrist_baseline_y;
	        float norm_ring_tip_y = iot_ring_tip_y - wrist_baseline_y;
	        float norm_pinky_tip_y = iot_pinky_tip_y - wrist_baseline_y;
	        float norm_thumb_tip_y = iot_thumb_tip_y - wrist_baseline_y;

	        printf("Index: %.2f, Middle: %.2f, Ring: %.2f, Pinky: %.2f, Thumb: %.2f\n",
	               norm_index_tip_y, norm_middle_tip_y, norm_ring_tip_y, norm_pinky_tip_y, norm_thumb_tip_y);

	        // Open Hand (Slightly Relaxed)
	            if ((fabs(norm_index_tip_y) > 0.3) &&
	                (fabs(norm_middle_tip_y) > 0.3) &&
	                (fabs(norm_ring_tip_y) > 0.3) &&
	                (fabs(norm_pinky_tip_y) > 0.2) &&
	                (fabs(norm_thumb_tip_y) > 0.2)) {
	                strcpy(iot_classification, "Open Hand");
	            }
	            // Fist (Adjusted to Avoid Confusion)
	            else if ((fabs(norm_index_tip_y) < 0.35) &&
	                     (fabs(norm_middle_tip_y) < 0.35) &&
	                     (fabs(norm_ring_tip_y) < 0.35) &&
	                     (fabs(norm_pinky_tip_y) < 0.35) &&
	                     (fabs(norm_thumb_tip_y) < 0.40)) {  // Allow thumb to be slightly lower
	                strcpy(iot_classification, "Fist");
	            }
	            // Thumbs Up (Adjusted to Match Your Values)
	            else if ((norm_thumb_tip_y < -0.5) &&  // Thumb is higher than the wrist significantly
	                     (norm_index_tip_y < 0.1) &&
	                     (norm_middle_tip_y < 0.1) &&
	                     (norm_ring_tip_y < 0.1) &&
	                     (norm_pinky_tip_y < 0.1)) {
	                strcpy(iot_classification, "Thumbs Up");
	            }
	            // Thumbs Down (Adjusted to Match Your Values)
	            // Thumbs Down (Better Matching)
	            else if ((norm_thumb_tip_y > 0.35) &&  // Ensure thumb is significantly higher
	                     (norm_index_tip_y < 0.2) &&
	                     (norm_middle_tip_y < 0.2) &&
	                     (norm_ring_tip_y < 0.2) &&
	                     (norm_pinky_tip_y < 0.2)) {
	                strcpy(iot_classification, "Thumbs Down");
	            }
	            // No Gesture Detected
	            else {
	                strcpy(iot_classification, "None");
	            }
	        } else {
	            strcpy(iot_classification, "No Hand");
	        }
	    }

	void iotc_thread_fct(ULONG arg)
	{
		for (int i = 0; i < 1000; i++)
		{
	        detect_hand_gesture(); // Call the gesture detection function

	        // Send gesture classification to IoTConnect
	        da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG classification,%s\r\n", iot_classification);
	        HAL_Delay(1000);

//			//Performance
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG cpu,%.2f,nn_fps,%.2f,nn_period,%.2f,inf_time,%.2f,hl_time,%.2f,disp_time,%.2f\r\n", (double)iot_cpu_percentage, (double)iot_nn_fps, (double)iot_nn_period_ms, (double)iot_pd_ms, (double)iot_hl_ms, (double)iot_disp_ms);
//			HAL_Delay(1000);
//
//			//Hand rotation
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG hand_rotation,%.2f\r\n", (double)iot_hand_rotation);
//			HAL_Delay(1000);
//
//			//Hand detection
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG nb_detect,%d,conf,%.2f,hand_valid,%d,hand_x,%.2f,hand_y,%.2f,hand_width,%.2f,hand_height,%.2f\r\n", iot_pd_hand_nb, (double)iot_pd_max_prob, iot_hand_valid, (double)iot_hand_x_center, (double)iot_hand_y_center, (double)iot_hand_width, (double)iot_hand_height);
//			HAL_Delay(1000);
////
//			//wrist_data
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG wrist_x,%.2f,wrist_y,%.2f\r\n", (double)iot_wrist_x, (double)iot_wrist_y);
//			HAL_Delay(1000);
//
//			//thumb_data1
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG thumb_cmc_x,%.2f,thumb_cmc_y,%.2f,thumb_mcp_x,%.2f,thumb_mcp_y,%.2f\r\n", (double)iot_thumb_cmc_x, (double)iot_thumb_cmc_y, (double)iot_thumb_mcp_x, (double)iot_thumb_mcp_y);
//			HAL_Delay(1000);
//
//			//thumb_data2
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG thumb_ip_x,%.2f,thumb_ip_y,%.2f,thumb_tip_x,%.2f,thumb_tip_y,%.2f\r\n", (double)iot_thumb_ip_x, (double)iot_thumb_ip_y, (double)iot_thumb_tip_x, (double)iot_thumb_tip_y);
//			HAL_Delay(1000);
//
//			//index_finger_data1
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG index_mcp_x,%.2f,index_mcp_y,%.2f,index_pip_x,%.2f,index_pip_y,%.2f\r\n", (double)iot_index_mcp_x, (double)iot_index_mcp_y, (double)iot_index_pip_x, (double)iot_index_pip_y);
//			HAL_Delay(1000);
//
//			//index_finger_data2
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG index_dip_x,%.2f,index_dip_y,%.2f,index_tip_x,%.2f,index_tip_y,%.2f\r\n", (double)iot_index_dip_x, (double)iot_index_dip_y, (double)iot_index_tip_x, (double)iot_index_tip_y);
//			HAL_Delay(1000);
//
//			//middle_finger_data1
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG middle_mcp_x,%.2f,middle_mcp_y,%.2f,middle_pip_x,%.2f,middle_pip_y,%.2f\r\n", (double)iot_middle_mcp_x, (double)iot_middle_mcp_y, (double)iot_middle_pip_x, (double)iot_middle_pip_y);
//			HAL_Delay(1000);
//
//			//middle_finger_data2
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG middle_dip_x,%.2f,middle_dip_y,%.2f,middle_tip_x,%.2f,middle_tip_y,%.2f\r\n", (double)iot_middle_dip_x, (double)iot_middle_dip_y, (double)iot_middle_tip_x, (double)iot_middle_tip_y);
//			HAL_Delay(1000);
//
//			//ring_finger_data1
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG ring_mcp_x,%.2f,ring_mcp_y,%.2f,ring_pip_x,%.2f,ring_pip_y,%.2f\r\n", (double)iot_ring_mcp_x, (double)iot_ring_mcp_y, (double)iot_ring_pip_x, (double)iot_ring_pip_y);
//			HAL_Delay(1000);
//
//			//ring_finger_data2
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG ring_dip_x,%.2f,ring_dip_y,%.2f,ring_tip_x,%.2f,ring_tip_y,%.2f\r\n", (double)iot_ring_dip_x, (double)iot_ring_dip_y, (double)iot_ring_tip_x, (double)iot_ring_tip_y);
//			HAL_Delay(1000);
//
//			//pinky_finger_data1
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG pinky_mcp_x,%.2f,pinky_mcp_y,%.2f,pinky_pip_x,%.2f,pinky_pip_y,%.2f\r\n", (double)iot_pinky_mcp_x, (double)iot_pinky_mcp_y, (double)iot_pinky_pip_x, (double)iot_pinky_pip_y);
//			HAL_Delay(1000);
//
//			//pinky_finger_data2
//			da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG pinky_dip_x,%.2f,pinky_dip_y,%.2f,pinky_tip_x,%.2f,pinky_tip_y,%.2f\r\n", (double)iot_pinky_dip_x, (double)iot_pinky_dip_y, (double)iot_pinky_tip_x, (double)iot_pinky_tip_y);

			HAL_Delay(2000);
		}
	}
	//iotc stop

	/**
	  * @brief  Main program
	  * @param  None
	  * @retval None
	  */
	int main(void)
	{
	  /* Power on ICACHE */
	  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_ICACTIVE_Msk;

	  /* Set back system and CPU clock source to HSI */
	  __HAL_RCC_CPUCLK_CONFIG(RCC_CPUCLKSOURCE_HSI);
	  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

	  HAL_Init();

	  SCB_EnableICache();

	#if defined(USE_DCACHE)
	  /* Power on DCACHE */
	  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
	  SCB_EnableDCache();
	#endif

	  return main_threadx();
	}

	void tx_application_define(void *first_unused_memory)
	{
	  const UINT priority = TX_MAX_PRIORITIES - 1;
	  const ULONG time_slice = 10;
	  int ret;

	  ret = tx_thread_create(&main_thread, "main", main_thread_fct, 0, main_thread_stack,
							 sizeof(main_thread_stack), priority, priority, time_slice, TX_AUTO_START);
	  assert(ret == 0);
	}

	static void NPURam_enable()
	{
	  __HAL_RCC_NPU_CLK_ENABLE();
	  __HAL_RCC_NPU_FORCE_RESET();
	  __HAL_RCC_NPU_RELEASE_RESET();

	  /* Enable NPU RAMs (4x448KB) */
	  __HAL_RCC_AXISRAM3_MEM_CLK_ENABLE();
	  __HAL_RCC_AXISRAM4_MEM_CLK_ENABLE();
	  __HAL_RCC_AXISRAM5_MEM_CLK_ENABLE();
	  __HAL_RCC_AXISRAM6_MEM_CLK_ENABLE();
	  __HAL_RCC_RAMCFG_CLK_ENABLE();
	  RAMCFG_HandleTypeDef hramcfg = {0};
	  hramcfg.Instance =  RAMCFG_SRAM3_AXI;
	  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
	  hramcfg.Instance =  RAMCFG_SRAM4_AXI;
	  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
	  hramcfg.Instance =  RAMCFG_SRAM5_AXI;
	  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
	  hramcfg.Instance =  RAMCFG_SRAM6_AXI;
	  HAL_RAMCFG_EnableAXISRAM(&hramcfg);
	}


	static void NPUCache_config()
	{
	  npu_cache_init();
	  npu_cache_enable();
	}

	static void Security_Config()
	{
	  __HAL_RCC_RIFSC_CLK_ENABLE();
	  RIMC_MasterConfig_t RIMC_master = {0};
	  RIMC_master.MasterCID = RIF_CID_1;
	  RIMC_master.SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
	  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_NPU, &RIMC_master);
	  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DMA2D, &RIMC_master);
	  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_DCMIPP, &RIMC_master);
	  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1 , &RIMC_master);
	  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2 , &RIMC_master);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI    , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC   , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2 , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
	}

	static void IAC_Config(void)
	{
	/* Configure IAC to trap illegal access events */
	  __HAL_RCC_IAC_CLK_ENABLE();
	  __HAL_RCC_IAC_FORCE_RESET();
	  __HAL_RCC_IAC_RELEASE_RESET();
	}

	void IAC_IRQHandler(void)
	{
	  while (1)
	  {
	  }
	}

	static void SystemClock_Config(void)
	{
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};

	  BSP_SMPS_Init(SMPS_VOLTAGE_OVERDRIVE);
	  HAL_Delay(1); /* Assuming Voltage Ramp Speed of 1mV/us --> 100mV increase takes 100us */

	  // Oscillator config already done in bootrom
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;

	  /* PLL1 = 64 x 25 / 2 = 800MHz */
	  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL1.PLLM = 2;
	  RCC_OscInitStruct.PLL1.PLLN = 25;
	  RCC_OscInitStruct.PLL1.PLLFractional = 0;
	  RCC_OscInitStruct.PLL1.PLLP1 = 1;
	  RCC_OscInitStruct.PLL1.PLLP2 = 1;

	  /* PLL2 = 64 x 125 / 8 = 1000MHz */
	  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL2.PLLM = 8;
	  RCC_OscInitStruct.PLL2.PLLFractional = 0;
	  RCC_OscInitStruct.PLL2.PLLN = 125;
	  RCC_OscInitStruct.PLL2.PLLP1 = 1;
	  RCC_OscInitStruct.PLL2.PLLP2 = 1;

	  /* PLL3 = (64 x 225 / 8) / (1 * 2) = 900MHz */
	  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL3.PLLM = 8;
	  RCC_OscInitStruct.PLL3.PLLN = 225;
	  RCC_OscInitStruct.PLL3.PLLFractional = 0;
	  RCC_OscInitStruct.PLL3.PLLP1 = 1;
	  RCC_OscInitStruct.PLL3.PLLP2 = 2;

	  /* PLL4 = (64 x 225 / 8) / (6 * 6) = 50 MHz */
	  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL4.PLLM = 8;
	  RCC_OscInitStruct.PLL4.PLLFractional = 0;
	  RCC_OscInitStruct.PLL4.PLLN = 225;
	  RCC_OscInitStruct.PLL4.PLLP1 = 6;
	  RCC_OscInitStruct.PLL4.PLLP2 = 6;

	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		while(1);
	  }

	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK |
									 RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
									 RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4 |
									 RCC_CLOCKTYPE_PCLK5);

	  /* CPU CLock (sysa_ck) = ic1_ck = PLL1 output/ic1_divider = 800 MHz */
	  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
	  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	  RCC_ClkInitStruct.IC1Selection.ClockDivider = 1;

	  /* AXI Clock (sysb_ck) = ic2_ck = PLL1 output/ic2_divider = 400 MHz */
	  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
	  RCC_ClkInitStruct.IC2Selection.ClockDivider = 2;

	  /* NPU Clock (sysc_ck) = ic6_ck = PLL2 output/ic6_divider = 1000 MHz */
	  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL2;
	  RCC_ClkInitStruct.IC6Selection.ClockDivider = 1;

	  /* AXISRAM3/4/5/6 Clock (sysd_ck) = ic11_ck = PLL3 output/ic11_divider = 900 MHz */
	  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL3;
	  RCC_ClkInitStruct.IC11Selection.ClockDivider = 1;

	  /* HCLK = sysb_ck / HCLK divider = 200 MHz */
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

	  /* PCLKx = HCLK / PCLKx divider = 200 MHz */
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
	  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
	  {
		while(1);
	  }

	  RCC_PeriphCLKInitStruct.PeriphClockSelection = 0;

	  /* XSPI1 kernel clock (ck_ker_xspi1) = HCLK = 200MHz */
	  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI1;
	  RCC_PeriphCLKInitStruct.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_HCLK;

	  /* XSPI2 kernel clock (ck_ker_xspi1) = HCLK =  200MHz */
	  RCC_PeriphCLKInitStruct.PeriphClockSelection |= RCC_PERIPHCLK_XSPI2;
	  RCC_PeriphCLKInitStruct.Xspi2ClockSelection = RCC_XSPI2CLKSOURCE_HCLK;

	  if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
	  {
		while (1);
	  }
	}

	static void CONSOLE_Config()
	{
	  GPIO_InitTypeDef gpio_init;

	  __HAL_RCC_USART1_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();

	 /* DISCO & NUCLEO USART1 (PE5/PE6) */
	  gpio_init.Mode      = GPIO_MODE_AF_PP;
	  gpio_init.Pull      = GPIO_PULLUP;
	  gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
	  gpio_init.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
	  gpio_init.Alternate = GPIO_AF7_USART1;
	  HAL_GPIO_Init(GPIOE, &gpio_init);

	  huart1.Instance          = USART1;
	  huart1.Init.BaudRate     = 115200;
	  huart1.Init.Mode         = UART_MODE_TX_RX;
	  huart1.Init.Parity       = UART_PARITY_NONE;
	  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits     = UART_STOPBITS_1;
	  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	  if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
		while (1);
	  }
	}

	static int main_threadx()
	{
	  _tx_initialize_kernel_setup();
	  tx_kernel_enter();
	  assert(0);

	  return -1;
	}

	static void main_thread_fct(ULONG arg)
	{
	  SystemClock_Config();

	  CONSOLE_Config();

	  //iotc start
	  int ret;
	  MX_USART2_UART_Init();
	  //iotc stop

	  NPURam_enable();
	  Fuse_Programming();

	  NPUCache_config();

	  /*** External RAM and NOR Flash *********************************************/
	  BSP_XSPI_RAM_Init(0);
	  BSP_XSPI_RAM_EnableMemoryMappedMode(0);

	  BSP_XSPI_NOR_Init_t NOR_Init;
	  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
	  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
	  BSP_XSPI_NOR_Init(0, &NOR_Init);
	  BSP_XSPI_NOR_EnableMemoryMappedMode(0);

	  /* Set all required IPs as secure privileged */
	  Security_Config();

	  IAC_Config();

	  /* Keep all IP's enabled during WFE so they can wake up CPU. Fine tune
	   * this if you want to save maximum power
	   */
	  LL_BUS_EnableClockLowPower(~0);
	  LL_MEM_EnableClockLowPower(~0);
	  LL_AHB1_GRP1_EnableClockLowPower(~0);
	  LL_AHB2_GRP1_EnableClockLowPower(~0);
	  LL_AHB3_GRP1_EnableClockLowPower(~0);
	  LL_AHB4_GRP1_EnableClockLowPower(~0);
	  LL_AHB5_GRP1_EnableClockLowPower(~0);
	  LL_APB1_GRP1_EnableClockLowPower(~0);
	  LL_APB1_GRP2_EnableClockLowPower(~0);
	  LL_APB2_GRP1_EnableClockLowPower(~0);
	  LL_APB4_GRP1_EnableClockLowPower(~0);
	  LL_APB4_GRP2_EnableClockLowPower(~0);
	  LL_APB5_GRP1_EnableClockLowPower(~0);
	  LL_MISC_EnableClockLowPower(~0);

	  app_run();

	//iotc start
	  const UINT iotc_priority = TX_MAX_PRIORITIES - 2;
	  const ULONG time_slice = 10;
	  ret = tx_thread_create(&iotc_thread, "iotc", iotc_thread_fct, 0, iotc_thread_stack,
							 sizeof(iotc_thread_stack), iotc_priority, iotc_priority, time_slice, TX_AUTO_START);
	  assert(ret == 0);
	//iotc stop
	}

	HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
	{
	  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};
	  HAL_StatusTypeDef ret;

	  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP;
	  RCC_PeriphCLKInitStruct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
	  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL2;
	  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockDivider = 3;
	  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
	  if (ret)
		return ret;

	  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CSI;
	  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
	  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockDivider = 40;
	  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
	  if (ret)
		return ret;

	  return HAL_OK;
	}

	void HAL_CACHEAXI_MspInit(CACHEAXI_HandleTypeDef *hcacheaxi)
	{
	  __HAL_RCC_CACHEAXIRAM_MEM_CLK_ENABLE();
	  __HAL_RCC_CACHEAXI_CLK_ENABLE();
	  __HAL_RCC_CACHEAXI_FORCE_RESET();
	  __HAL_RCC_CACHEAXI_RELEASE_RESET();
	}

	void HAL_CACHEAXI_MspDeInit(CACHEAXI_HandleTypeDef *hcacheaxi)
	{
	  __HAL_RCC_CACHEAXIRAM_MEM_CLK_DISABLE();
	  __HAL_RCC_CACHEAXI_CLK_DISABLE();
	  __HAL_RCC_CACHEAXI_FORCE_RESET();
	}

	#ifdef  USE_FULL_ASSERT

	/**
	  * @brief  Reports the name of the source file and the source line number
	  *         where the assert_param error has occurred.
	  * @param  file: pointer to the source file name
	  * @param  line: assert_param error line source number
	  * @retval None
	  */
	void assert_failed(uint8_t* file, uint32_t line)
	{
	  printf("Assertion failed: %s, line %d\n", file, line);
	  NVIC_SystemReset(); // Reset the system instead of infinite loop
	}
	#endif

	/* Allow to debug with cache enable */
	__attribute__ ((section (".keep_me"))) void app_clean_invalidate_dbg()
	{
	  SCB_CleanInvalidateDCache();
	}
