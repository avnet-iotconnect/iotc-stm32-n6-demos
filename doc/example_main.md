### Step 1: Replace main.c with below
```
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

#include "da16k_comm.h"

extern int __uncached_bss_start__;
extern int __uncached_bss_end__;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static TX_THREAD main_thread;
static uint8_t main_tread_stack[4096];

static TX_THREAD iotc_thread;
static uint8_t iotc_tread_stack[4096];

static void SystemClock_Config(void);
static void NPURam_enable();
static void NPUCache_config();
static void Security_Config();
static void IAC_Config();
static void CONSOLE_Config(void);
static void Setup_Mpu(void);
static int main_threadx(void);
static void main_thread_fct(ULONG arg);


#define TX_BUFFER_SIZE  512
#define RX_BUFFER_SIZE  256
static char command[TX_BUFFER_SIZE] = { 0 };
static char response[RX_BUFFER_SIZE] = { 0 };

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

  HAL_UART_Transmit(&huart2, (uint8_t* )command, command_size, timeout_ms);

  do {
	USART_STATUS = HAL_UART_Receive(&huart2, (uint8_t* )&response[i], 1, timeout_ms);
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

void iotc_thread_fct(ULONG arg) {

	printf("SENDING IOTC MESSAGES\n");
	double version = 123456;
	char key[] = "version";
	for (int i = 0; i < 1000; i++) {
		da16k_send_msg_direct_num(key, version);
		version++;
		HAL_Delay(5000);
	}
}

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

  Setup_Mpu();

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

  ret = tx_thread_create(&main_thread, "main", main_thread_fct, 0, main_tread_stack,
                         sizeof(main_tread_stack), priority, priority, time_slice, TX_AUTO_START);
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

static void Setup_Mpu()
{
  MPU_Attributes_InitTypeDef attr;
  MPU_Region_InitTypeDef region;

  attr.Number = MPU_ATTRIBUTES_NUMBER0;
  attr.Attributes = MPU_NOT_CACHEABLE;
  HAL_MPU_ConfigMemoryAttributes(&attr);

  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER0;
  region.BaseAddress = (uint32_t)&__uncached_bss_start__;
  region.LimitAddress = (uint32_t)&__uncached_bss_end__ - 1;
  region.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  region.AccessPermission = MPU_REGION_ALL_RW;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  HAL_MPU_ConfigRegion(&region);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
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
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_VENC , &RIMC_master);
  HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_OTG1 , &RIMC_master);

  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_NPU , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DMA2D , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_CSI    , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_DCMIPP , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_VENC , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
  HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_OTG1HS , RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
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

  /* PLL4 in bypass mode.
   * Keep PLL4 output active to allow IC17 and IC18 clock source switch from PLL4 (reset value) to PLL2 and PLL1 (see
   * RM0486 Clock switches and gating).
   */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_BYPASS;

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

static void DMA2D_Config()
{
  __HAL_RCC_DMA2D_CLK_ENABLE();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();
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
  int ret;

  SystemClock_Config();

  CONSOLE_Config();
  MX_USART2_UART_Init();

  NPURam_enable();
  Fuse_Programming();

  DMA2D_Config();

  NPUCache_config();

  /*** External RAM and NOR Flash *********************************************/
  BSP_XSPI_RAM_Init(0);
  BSP_XSPI_RAM_EnableMemoryMappedMode(0);

  BSP_XSPI_NOR_Init_t NOR_Init;
  NOR_Init.InterfaceMode = BSP_XSPI_NOR_OPI_MODE;
  NOR_Init.TransferRate = BSP_XSPI_NOR_DTR_TRANSFER;
  BSP_XSPI_NOR_Init(0, &NOR_Init);
  BSP_XSPI_NOR_EnableMemoryMappedMode(0);

  ret = BSP_LED_Init(LED_GREEN);
  assert(ret == BSP_ERROR_NONE);

  ret = BSP_LED_Init(LED_RED);
  assert(ret == BSP_ERROR_NONE);

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

  const UINT iotc_priority = TX_MAX_PRIORITIES - 2;
  const ULONG time_slice = 10;
  ret = tx_thread_create(&iotc_thread, "iotc", iotc_thread_fct, 0, iotc_tread_stack,
                         sizeof(iotc_tread_stack), iotc_priority, iotc_priority, time_slice, TX_AUTO_START);
  assert(ret == 0);

  /* keep main thread as idle thread */
  //while (1)
   // ;
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

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  assert(hpcd->Instance == USB1_OTG_HS);

  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable the VDD33USB independent USB 33 voltage monitor */
  HAL_PWREx_EnableVddUSBVMEN();

  /* Wait until VDD33USB is ready */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_USB33RDY) == 0U);

  /* Enable VDDUSB supply */
  HAL_PWREx_EnableVddUSB();

  /* Enable USB1 OTG clock */
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

  /* Set FSEL to 24 Mhz */
  USB1_HS_PHYC->USBPHYC_CR &= ~(0x7U << 0x4U);
  USB1_HS_PHYC->USBPHYC_CR |= (0x2U << 0x4U);

  /* Enable USB1 OTG PHY clock */
  __HAL_RCC_USB1_OTG_HS_PHY_CLK_ENABLE();

  HAL_NVIC_SetPriority(USB1_OTG_HS_IRQn, 6U, 0U);

  /* Enable USB OTG interrupt */
  HAL_NVIC_EnableIRQ(USB1_OTG_HS_IRQn);
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
  UNUSED(file);
  UNUSED(line);
  __BKPT(0);
  while (1)
  {
  }
}
#endif

/* Allow to debug with cache enable */
__attribute__ ((section (".keep_me"))) void app_clean_invalidate_dbg()
{
  SCB_CleanInvalidateDCache();
}
``` 
> [!NOTE]
> If you do not want to report UVC data, you only need Step 1.

### Step 2: Report UVC data to IOTCONNECT platform 
* UVC application data in x-cube-n6-ai-h264-usb-uvc-v1.0.0 example:
  * number of people detected: nb_detect
  * person in frame x coordinate: x_center
  * person in frame y coordinate: y_center

* Modify the function *dp_thread_fct* below in app.c.

```
int32_t nb_detect = 0;
float32_t x_center = 0;
float32_t y_center = 0;

static void dp_thread_fct(ULONG arg)
{
#if POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V2_UF
  yolov2_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V5_UU
  yolov5_pp_static_param_t pp_params;
#elif POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UF || POSTPROCESS_TYPE == POSTPROCESS_OD_YOLO_V8_UI
  yolov8_pp_static_param_t pp_params;
#else
    #error "PostProcessing type not supported"
#endif
  od_pp_out_t pp_output;
  uint32_t total_ts;
  void *pp_input;
  int is_dp_done;
  uint32_t ts;
  int ret;

  /* setup post process */
  app_postprocess_init(&pp_params);
  while (1)
  {
    uint8_t *output_buffer;

    output_buffer = bqueue_get_ready(&nn_output_queue);
    assert(output_buffer);
    total_ts = HAL_GetTick();

    /* Do post process */
    ts = HAL_GetTick();
    pp_input = (void *) output_buffer;
    pp_output.pOutBuff = NULL;
    ret = app_postprocess_run((void * []){pp_input}, 1, &pp_output, &pp_params);
    assert(ret == AI_OD_POSTPROCESS_ERROR_NO);
    time_stat_update(&stat_info.nn_pp_time, HAL_GetTick() - ts);
    cpuload_update(&cpu_load);

    /* compose + encode + send */
    is_dp_done = display_new_frame(&pp_output);

    nb_detect = pp_output.nb_detect;
    x_center = pp_output.pOutBuff->x_center;
    y_center = pp_output.pOutBuff->y_center;

    if (is_dp_done)
      time_stat_update(&stat_info.disp_total_time, HAL_GetTick() - total_ts);

    bqueue_put_free(&nn_output_queue);
  }
}
```
* Use function da16k_send_msg_direct_float(key, value) in main.c to report UVC data to IOTCONNECT