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
#include "cmw_camera.h"
#include "stm32n6570_discovery_bus.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery_xspi.h"
#include "stm32n6570_discovery.h"
#include "stm32_lcd.h"
#include "app_fuseprogramming.h"
#include "stm32_lcd_ex.h"
#include "app_postprocess.h"
#include "ll_aton_runtime.h"
#include "app_cam.h"
#include "main.h"
#include <stdio.h>
#include "stm32n6xx_hal_rif.h"
#include "app_config.h"
#include "crop_img.h"
#include "stlogo.h"
#include "utils.h"

#include "da16k_comm.h"

#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
  #include "display_mpe.h"
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  #include "display_spe.h"
#else
  #error "PostProcessing type not supported"
#endif

#define MAX_NUMBER_OUTPUT 5
#define IOTC_INTERVAL 5000

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

/* Lcd Background area */
Rectangle_TypeDef lcd_bg_area = {
#if ASPECT_RATIO_MODE == ASPECT_RATIO_CROP || ASPECT_RATIO_MODE == ASPECT_RATIO_FIT
  .X0 = (LCD_FG_WIDTH - LCD_FG_HEIGHT) / 2,
#else
  .X0 = 0,
#endif
  .Y0 = 0,
  .XSize = 0,
  .YSize = 0,
};

/* Lcd Foreground area */
Rectangle_TypeDef lcd_fg_area = {
  .X0 = 0,
  .Y0 = 0,
  .XSize = LCD_FG_WIDTH,
  .YSize = LCD_FG_HEIGHT,
};

#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
  mpe_yolov8_pp_static_param_t pp_params;
  mpe_pp_out_t pp_output;
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  spe_movenet_pp_static_param_t pp_params;
  spe_pp_out_t pp_output;
#else
  #error "PostProcessing type not supported"
#endif

volatile int32_t cameraFrameReceived;
uint8_t *nn_in;
BSP_LCD_LayerConfig_t LayerConfig = {0};

#define ALIGN_TO_16(value) (((value) + 15) & ~15)

/* for models not multiple of 16; needs a working buffer */
#if (NN_WIDTH * NN_BPP) != ALIGN_TO_16(NN_WIDTH * NN_BPP)
#define DCMIPP_OUT_NN_LEN (ALIGN_TO_16(NN_WIDTH * NN_BPP) * NN_HEIGHT)
#define DCMIPP_OUT_NN_BUFF_LEN (DCMIPP_OUT_NN_LEN + 32 - DCMIPP_OUT_NN_LEN%32)

__attribute__ ((aligned (32)))
uint8_t dcmipp_out_nn[DCMIPP_OUT_NN_BUFF_LEN];
#else
uint8_t *dcmipp_out_nn;
#endif

/* Lcd Background Buffer */
__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t lcd_bg_buffer[800 * 480 * 2];
/* Lcd Foreground Buffer */
__attribute__ ((section (".psram_bss")))
__attribute__ ((aligned (32)))
uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT * 2];
static int lcd_fg_buffer_rd_idx;

static void SystemClock_Config(void);
static void NPURam_enable(void);
static void NPUCache_config(void);
static void Display_NetworkOutput(void *p_postprocess, uint32_t inference_ms);
static void LCD_init(void);
static void Security_Config(void);
static void set_clk_sleep_mode(void);
static void IAC_Config(void);
static void Display_WelcomeScreen(void);





UART_HandleTypeDef huart2;
extern da16k_err_t da16k_at_send_formatted_raw_no_crlf(const char *format, ...);
int x[13] = {0};
int y[13] = {0};
int color[13] = {0};
char iotc_color_str[13][20] = {0};
static uint32_t iotc_inference = 0;
static uint32_t last_send_time = 0;

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
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
	  while (1);
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
	  while (1);
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
	  while (1);
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
	  while (1);
  }
}

static void convert_color_to_str(int input_color, char* color_str) {
  if (input_color == 0) {
	  return;
  }
  else if (input_color == COLOR_HEAD) {
	  strcpy(color_str, "COLOR_HEAD_GREEN");
  }
  else if (input_color == COLOR_ARMS) {
	  strcpy(color_str, "COLOR_ARMS_BLUE");
  }
  else if (input_color == COLOR_TRUNK) {
	  strcpy(color_str, "COLOR_TRUNK_MAGENTA");
  }
  else if (input_color == COLOR_LEGS) {
	  strcpy(color_str, "COLOR_LEGS_ORANGE");
  }
  else {
	  strcpy(color_str, "UNKNOWN");
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

  SCB_EnableICache();

#if defined(USE_DCACHE)
  /* Power on DCACHE */
  MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCACTIVE_Msk;
  SCB_EnableDCache();
#endif

  SystemClock_Config();

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

  MX_USART2_UART_Init();

  /* Set all required IPs as secure privileged */
  Security_Config();

  IAC_Config();
  set_clk_sleep_mode();

  /*** NN Init ****************************************************************/
  LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(Default);
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_Default();
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_Default();

  nn_in = (uint8_t *) LL_Buffer_addr_start(&nn_in_info[0]);
  float32_t *nn_out[MAX_NUMBER_OUTPUT];
  int32_t nn_out_len[MAX_NUMBER_OUTPUT];

  int number_output = 0;

  /* Count number of outputs */
  while (nn_out_info[number_output].name != NULL)
  {
    number_output++;
  }
  assert(number_output <= MAX_NUMBER_OUTPUT);

  for (int i = 0; i < number_output; i++)
  {
    nn_out[i] = (float32_t *) LL_Buffer_addr_start(&nn_out_info[i]);
    nn_out_len[i] = LL_Buffer_len(&nn_out_info[i]);
  }

  uint32_t nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  uint32_t pitch_nn = 0;

  UNUSED(nn_in_len);

  /*** Post Processing Init ***************************************************/
  app_postprocess_init(&pp_params);

  /*** Camera Init ************************************************************/

  CAM_Init(&lcd_bg_area.XSize, &lcd_bg_area.YSize, &pitch_nn);

  LCD_init();

  /* da16k module init */
  da16k_cfg_t cfg = {0};
  da16k_init(&cfg);

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(lcd_bg_buffer, CMW_MODE_CONTINUOUS);

  /*** App Loop ***************************************************************/
  while (1)
  {
	//IOTCONNECT telemetry data init
	memset(x, 0, sizeof(x));
	memset(y, 0, sizeof(y));
	memset(color, 0, sizeof(color));
	iotc_inference = 0;

    CAM_IspUpdate();

    if (pitch_nn != (NN_WIDTH * NN_BPP))
    {
      /* Start NN camera single capture Snapshot */
      CAM_NNPipe_Start(dcmipp_out_nn, CMW_MODE_SNAPSHOT);
    }
    else
    {
      /* Start NN camera single capture Snapshot */
      CAM_NNPipe_Start(nn_in, CMW_MODE_SNAPSHOT);
    }

    while (cameraFrameReceived == 0) {};
    cameraFrameReceived = 0;

    uint32_t ts[2] = { 0 };

    if (pitch_nn != (NN_WIDTH * NN_BPP))
    {
      SCB_InvalidateDCache_by_Addr(dcmipp_out_nn, sizeof(dcmipp_out_nn));
      img_crop(dcmipp_out_nn, nn_in, pitch_nn, NN_WIDTH, NN_HEIGHT, NN_BPP);
      SCB_CleanInvalidateDCache_by_Addr(nn_in, nn_in_len);
    }

    ts[0] = HAL_GetTick();
    /* run ATON inference */
    LL_ATON_RT_Main(&NN_Instance_Default);
    ts[1] = HAL_GetTick();

    int32_t ret = app_postprocess_run((void **) nn_out, number_output, &pp_output, &pp_params);
    assert(ret == 0);

    Display_NetworkOutput(&pp_output, ts[1] - ts[0]);
    /* Discard nn_out region (used by pp_input and pp_outputs variables) to avoid Dcache evictions during nn inference */
    for (int i = 0; i < number_output; i++)
    {
      float32_t *tmp = nn_out[i];
      SCB_InvalidateDCache_by_Addr(tmp, nn_out_len[i]);
    }

    //IOTCONNECT
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_send_time) < IOTC_INTERVAL) {
      continue;
    }
    last_send_time = current_time;

    //only send data to iotconnect when at least three points showing up in order to avoid floating point data
    if ( !((x[0] == 0 && y[0] == 0) || (x[1] == 0 && y[1] == 0) || (x[2] == 0 && y[2] == 0)) ) {
      //printf("Sending the message to IOTCONNECT...\r\n");
      for (int i = 0; i < 13; i++) {
        convert_color_to_str(color[i], &iotc_color_str[i][0]);
      }
	  da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point1_x,%d,point1_y,%d,point1_color,%s,point2_x,%d,point2_y,%d,point2_color,%s,inference,%d\r\n",
										   x[0], y[0], &iotc_color_str[0][0], x[1], y[1], &iotc_color_str[1][0], iotc_inference);

	  if (!((x[2] == 0 && y[2] == 0) && (x[3] == 0 && y[3] == 0))) {
		HAL_Delay(100);
	    da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point3_x,%d,point3_y,%d,point3_color,%s,point4_x,%d,point4_y,%d,point4_color,%s\r\n",
	  										   x[2], y[2], &iotc_color_str[2][0], x[3], y[3], &iotc_color_str[3][0]);
	  }

	  if (!((x[4] == 0 && y[4] == 0) && (x[5] == 0 && y[5] == 0))) {
		HAL_Delay(100);
	    da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point5_x,%d,point5_y,%d,point5_color,%s,point6_x,%d,point6_y,%d,point6_color,%s\r\n",
	  										   x[4], y[4], &iotc_color_str[4][0], x[5], y[5], &iotc_color_str[5][0]);
	  }
	  if (!((x[6] == 0 && y[6] == 0) && (x[7] == 0 && y[7] == 0))) {
		HAL_Delay(100);
	    da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point7_x,%d,point7_y,%d,point7_color,%s,point8_x,%d,point8_y,%d,point8_color,%s\r\n",
	  	  										   x[6], y[6], &iotc_color_str[6][0], x[7], y[7], &iotc_color_str[7][0]);
	  }
	  if (!((x[8] == 0 && y[8] == 0) && (x[9] == 0 && y[9] == 0))) {
		HAL_Delay(100);
	    da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point9_x,%d,point9_y,%d,point9_color,%s,point10_x,%d,point10_y,%d,point10_color,%s\r\n",
	  	  	  										   x[8], y[8], &iotc_color_str[8][0], x[9], y[9], &iotc_color_str[9][0]);
	  }
	  if (!((x[10] == 0 && y[10] == 0) && (x[11] == 0 && y[11] == 0))) {
		HAL_Delay(100);
	    da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG point11_x,%d,point11_y,%d,point11_color,%s,point12_x,%d,point12_y,%d,point12_color,%s\r\n",
	  	  	  										   x[10], y[10], &iotc_color_str[10][0], x[11], y[11], &iotc_color_str[11][0]);
	  }
    }
    //IOTCONNECT to receive C2D message
	da16k_cmd_t current_cmd = {0};
	if ((da16k_get_cmd(&current_cmd) == DA16K_SUCCESS) && current_cmd.command) {
	  //USE current_cmd.command & current_cmd.parameters here
	  printf("/IOTCONNECT command is %s\r\n",current_cmd.command);
	  if (current_cmd.parameters) {
	    printf("/IOTCONNECT command->parameter is %s\r\n",current_cmd.parameters);
		da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG iotc_cmd,%s,iotc_cmd_parameter,%s\r\n", current_cmd.command, current_cmd.parameters);
	  }
      da16k_destroy_cmd(current_cmd);
    }
  }
}

static void NPURam_enable(void)
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

static void set_clk_sleep_mode(void)
{
  /*** Enable sleep mode support during NPU inference *************************/
  /* Configure peripheral clocks to remain active during sleep mode */
  /* Keep all IP's enabled during WFE so they can wake up CPU. Fine tune
   * this if you want to save maximum power
   */
  __HAL_RCC_XSPI1_CLK_SLEEP_ENABLE();    /* For display frame buffer */
  __HAL_RCC_XSPI2_CLK_SLEEP_ENABLE();    /* For NN weights */
  __HAL_RCC_NPU_CLK_SLEEP_ENABLE();      /* For NN inference */
  __HAL_RCC_CACHEAXI_CLK_SLEEP_ENABLE(); /* For NN inference */
  __HAL_RCC_LTDC_CLK_SLEEP_ENABLE();     /* For display */
  __HAL_RCC_DMA2D_CLK_SLEEP_ENABLE();    /* For display */
  __HAL_RCC_DCMIPP_CLK_SLEEP_ENABLE();   /* For camera configuration retention */
  __HAL_RCC_CSI_CLK_SLEEP_ENABLE();      /* For camera configuration retention */

  __HAL_RCC_FLEXRAM_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM1_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM2_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM3_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_SLEEP_ENABLE(); 

}

static void NPUCache_config(void)
{

  npu_cache_init();
  npu_cache_enable();
}

static void Security_Config(void)
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

/* Display functions */
static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < (int)lcd_bg_area.X0)
    *x = lcd_bg_area.X0;
  if (*y < (int)lcd_bg_area.Y0)
    *y = lcd_bg_area.Y0;
  if (*x >= lcd_bg_area.X0 + lcd_bg_area.XSize)
    *x = lcd_bg_area.X0 + lcd_bg_area.XSize - 1;
  if (*y >= lcd_bg_area.Y0 + lcd_bg_area.YSize)
    *y = lcd_bg_area.Y0 + lcd_bg_area.YSize - 1;

  return (xi != *x) || (yi != *y);
}

static void convert_length(float32_t wi, float32_t hi, int *wo, int *ho)
{
  *wo = lcd_bg_area.XSize * wi;
  *ho = lcd_bg_area.YSize * hi;
}

static void convert_point(float32_t xi, float32_t yi, int *xo, int *yo)
{
  *xo = lcd_bg_area.XSize * xi + lcd_bg_area.X0;
  *yo = lcd_bg_area.YSize * yi + lcd_bg_area.Y0;
}

static void Display_binding_line(int x0, int y0, int x1, int y1, uint32_t color)
{
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);

  UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
}

/**
* @brief Display Neural Network output classification results as well as other performances informations
*
* @param p_postprocess pointer to postprocessing output
* @param inference_ms inference time in ms
*/
static void Display_NetworkOutput(void *p_postprocess, uint32_t inference_ms)
{
#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
  mpe_pp_outBuffer_t *rois = ((mpe_pp_out_t *) p_postprocess)->pOutBuff;
  uint32_t nb_rois = ((mpe_pp_out_t *) p_postprocess)->nb_detect;
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  spe_pp_outBuffer_t *roi = ((spe_pp_out_t *) p_postprocess)->pOutBuff;
#endif
  int ret;

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
  assert(ret == HAL_OK);

  /* Draw bounding boxes */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */
#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
  for (int i = 0; i < nb_rois; i++)
    Display_mpe_Detection(&rois[i]);
  UTIL_LCD_SetBackColor(0x40000000);
  UTIL_LCDEx_PrintfAt(0, LINE(2), CENTER_MODE, "Objects %u", nb_rois);
  UTIL_LCDEx_PrintfAt(0, LINE(20), CENTER_MODE, "Inference: %ums", inference_ms);
  UTIL_LCD_SetBackColor(0);
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  Display_spe_Detection(roi);
  UTIL_LCD_SetBackColor(0x40000000);
  UTIL_LCDEx_PrintfAt(0, LINE(20), CENTER_MODE, "Inference: %ums", inference_ms);
  UTIL_LCD_SetBackColor(0);
  //FOR IOTCONNECT MSG
  iotc_inference = inference_ms;
#endif

  Display_WelcomeScreen();

  SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_FRAMEBUFFER_SIZE);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
  assert(ret == HAL_OK);
  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

static void LCD_init()
{
  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);

  /* Preview layer Init */
  LayerConfig.X0          = lcd_bg_area.X0;
  LayerConfig.Y0          = lcd_bg_area.Y0;
  LayerConfig.X1          = lcd_bg_area.X0 + lcd_bg_area.XSize;
  LayerConfig.Y1          = lcd_bg_area.Y0 + lcd_bg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_RGB565;
  LayerConfig.Address     = (uint32_t) lcd_bg_buffer;

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &LayerConfig);

  LayerConfig.X0 = lcd_fg_area.X0;
  LayerConfig.Y0 = lcd_fg_area.Y0;
  LayerConfig.X1 = lcd_fg_area.X0 + lcd_fg_area.XSize;
  LayerConfig.Y1 = lcd_fg_area.Y0 + lcd_fg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_ARGB4444;
  LayerConfig.Address = (uint32_t) lcd_fg_buffer; /* External XSPI1 PSRAM */

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &LayerConfig);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(LTDC_LAYER_2);
  UTIL_LCD_Clear(0x00000000);
  UTIL_LCD_SetFont(&Font20);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);

#if POSTPROCESS_TYPE == POSTPROCESS_MPE_YOLO_V8_UF
  Display_mpe_InitFunctions(clamp_point,
                            convert_length,
                            convert_point,
                            Display_binding_line);
#elif POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  Display_spe_InitFunctions(clamp_point,
                            convert_length,
                            convert_point,
                            Display_binding_line);
#endif
}

/**
 * @brief Displays a Welcome screen
 */
static void Display_WelcomeScreen(void)
{
  static uint32_t t0 = 0;
  if (t0 == 0)
    t0 = HAL_GetTick();

  if (HAL_GetTick() - t0 < 4000)
  {
    /* Draw logo */
    UTIL_LCD_FillRGBRect(300, 100, (uint8_t *) stlogo, 200, 107);
    
    /* Display welcome message */
    UTIL_LCD_SetBackColor(0x40000000);
    UTIL_LCDEx_PrintfAt(0, LINE(16), CENTER_MODE, "Pose estimation");
    UTIL_LCDEx_PrintfAt(0, LINE(17), CENTER_MODE, WELCOME_MSG_1);
    UTIL_LCDEx_PrintfAt(0, LINE(18), CENTER_MODE, WELCOME_MSG_2);
    UTIL_LCD_SetBackColor(0);
  }
}

/**
  * @brief  DCMIPP Clock Config for DCMIPP.
  * @param  hcsi  DCMIPP Handle
  *         Being __weak it can be overwritten by the application
  * @retval HAL_status
  */
HAL_StatusTypeDef MX_DCMIPP_ClockConfig(DCMIPP_HandleTypeDef *hdcmipp)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};
  HAL_StatusTypeDef ret = HAL_OK;

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DCMIPP;
  RCC_PeriphCLKInitStruct.DcmippClockSelection = RCC_DCMIPPCLKSOURCE_IC17;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockSelection = RCC_ICCLKSOURCE_PLL2;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC17].ClockDivider = 3;
  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  if (ret)
  {
    return ret;
  }

  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CSI;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_PeriphCLKInitStruct.ICSelection[RCC_IC18].ClockDivider = 40;
  ret = HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  if (ret)
  {
    return ret;
  }

  return ret;
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct = {0};

  /* Ensure VDDCORE=0.9V before increasing the system frequency */
  BSP_SMPS_Init(SMPS_VOLTAGE_OVERDRIVE);

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
