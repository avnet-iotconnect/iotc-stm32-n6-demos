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
 inference_ms
 number of objects:  Objects
 Bounding Box Information
     x_center, y_center: The center of the detected object.
    width, height: The width and height of the bounding box.
    pMask: A segmentation mask with probability values.
    class_index: The detected object's class.
 Confidence
    pMask
        This indicates confidence per-pixel for segmentation.
        Could be average   d per object or reported as a probability per bounding box.
    YOLOv8 Object Detection Thresholds
 Preprocessed Image Resolution

 Aspect Ratio Handling

 Camera Frame Processing Time
 System Load Information

 Neural Network Deployment Info
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

// iotc-start
#include "stlogo.h"
#include "da16k_comm.h"
#include "da16k_uart.h" // For UART
#include "stm32n6xx_hal_uart.h" // STM32 HAL UART header
#include <stdbool.h>

#define MAX_TRACKED_OBJECTS 10
#define IOU_THRESHOLD 0.5f  // Intersection-over-Union threshold for matching

typedef struct
{
    int id;                // Persistent ID (1-10)
    int class_index;       // Object class
    float confidence;      // Confidence score
    uint32_t x, y, w, h;   // Bounding box
    bool active;           // Is this slot currently tracking an object?
} TrackedObject;

static TrackedObject tracked_objects[MAX_TRACKED_OBJECTS] = {0}; // Initialize all IDs to 0

static int assign_id_to_object(int class_index, float confidence, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    // Try to find an existing match (IoU-based matching)
    for (int i = 0; i < MAX_TRACKED_OBJECTS; i++)
    {
        if (tracked_objects[i].active)
        {
            float iou = calculate_iou(tracked_objects[i], x, y, w, h);
            if (iou > IOU_THRESHOLD) // If it's a known object, update its info
            {
                tracked_objects[i].class_index = class_index;
                tracked_objects[i].confidence = confidence;
                tracked_objects[i].x = x;
                tracked_objects[i].y = y;
                tracked_objects[i].w = w;
                tracked_objects[i].h = h;
                return tracked_objects[i].id;
            }
        }
    }

    // If no existing match, assign a new ID if space is available
    for (int i = 0; i < MAX_TRACKED_OBJECTS; i++)
    {
        if (!tracked_objects[i].active)  // Find an empty slot
        {
            tracked_objects[i].id = i + 1; // IDs 1-10
            tracked_objects[i].class_index = class_index;
            tracked_objects[i].confidence = confidence;
            tracked_objects[i].x = x;
            tracked_objects[i].y = y;
            tracked_objects[i].w = w;
            tracked_objects[i].h = h;
            tracked_objects[i].active = true;
            return tracked_objects[i].id;
        }
    }

    // If no available slot, return -1 (ignore object)
    return -1;
}

/**
 * @brief Calculates the Intersection-over-Union (IoU) between two bounding boxes.
 */
static float calculate_iou(TrackedObject existing, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    uint32_t xA = MAX(existing.x, x);
    uint32_t yA = MAX(existing.y, y);
    uint32_t xB = MIN(existing.x + existing.w, x + w);
    uint32_t yB = MIN(existing.y + existing.h, y + h);

    uint32_t interArea = MAX(0, xB - xA) * MAX(0, yB - yA);
    uint32_t boxAArea = existing.w * existing.h;
    uint32_t boxBArea = w * h;

    return (float) interArea / (boxAArea + boxBArea - interArea);
}


#define TX_BUFFER_SIZE 512
UART_HandleTypeDef huart2;
extern da16k_err_t da16k_at_send_formatted_raw_no_crlf(const char *format, ...);
int confidence_threshold = 55; // Default confidence threshold (70%)
// iotc-stop

CLASSES_TABLE;

#define MAX_NUMBER_OUTPUT 5

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

#define NUMBER_COLORS 10
const uint32_t colors[NUMBER_COLORS] = {
    UTIL_LCD_COLOR_GREEN,
    UTIL_LCD_COLOR_RED,
    UTIL_LCD_COLOR_CYAN,
    UTIL_LCD_COLOR_MAGENTA,
    UTIL_LCD_COLOR_YELLOW,
    UTIL_LCD_COLOR_GRAY,
    UTIL_LCD_COLOR_BLACK,
    UTIL_LCD_COLOR_BROWN,
    UTIL_LCD_COLOR_BLUE,
    UTIL_LCD_COLOR_ORANGE
};

#if POSTPROCESS_TYPE == POSTPROCESS_ISEG_YOLO_V8_UI
yolov8_seg_pp_static_param_t pp_params;
#else
    #error "PostProcessing type not supported"
#endif

volatile int32_t cameraFrameReceived;
uint8_t *nn_in;
BSP_LCD_LayerConfig_t LayerConfig = {0};
iseg_postprocess_out_t pp_output;

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
static void Display_NetworkOutput(iseg_postprocess_out_t *p_postprocess, uint32_t inference_ms);
static void LCD_init(void);
static void Security_Config(void);
static void set_clk_sleep_mode(void);
static void IAC_Config(void);
static void Display_WelcomeScreen(void);

/***IoTC-start1***/
UART_HandleTypeDef huart2;
#define TX_BUFFER_SIZE  512
#define RX_BUFFER_SIZE  512
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
/***IoTC-stop1***/

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
// iotc-start
  MX_USART2_UART_Init(); // Initialize UART
// iotc-stop

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

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(lcd_bg_buffer, CMW_MODE_CONTINUOUS);

  /*** App Loop ***************************************************************/
  while (1)
    //iotc-start
  {
      da16k_cmd_t current_cmd = {0};

      // Attempt to receive a command
      HAL_Delay(50); // small 50ms pause

      da16k_err_t get_ret = da16k_get_cmd(&current_cmd);
      if (get_ret == DA16K_SUCCESS)
      {
          printf("Got command: '%s'\n",
                 current_cmd.command ? current_cmd.command : "(null)");
          printf("Params: '%s'\n",
                 current_cmd.parameters ? current_cmd.parameters : "(null)");

          // Check if it is SET_CONFIDENCE_THRESHOLD
          #define CMD_STR "SET_CONFIDENCE_THRESHOLD"
          if (current_cmd.command &&
              strcmp(current_cmd.command, CMD_STR) == 0)
          {
              // We got the right command
              if (current_cmd.parameters)
              {
                  int new_threshold = atoi(current_cmd.parameters);
                  if (new_threshold >= 0 && new_threshold <= 100) {
                      confidence_threshold = new_threshold;
                      printf("Updated threshold to %d%%\n", confidence_threshold);

                      // Acknowledge
                      da16k_at_send_formatted_raw_no_crlf(
                        "AT+NWICMSG threshold_updated,%d\r\n", confidence_threshold
                      );
                  } else {
                      printf("Invalid threshold: %d\n", new_threshold);
                  }
              }
          }
          else
          {
              printf("Unrecognized command: '%s'\n",
                     current_cmd.command ? current_cmd.command : "(null)");
          }

          // Clean up
          da16k_destroy_cmd(current_cmd);
      }
      else if (get_ret == DA16K_NO_CMDS)
      {
          // No new commands from the module
          // ...
      }
      else
      {
          // Some error occurred
          printf("Error retrieving command: %d\n", get_ret);
      }
// iotc-stop

  {
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

//iotc-start
    // Send classification results to IoTConnect
    if ((nn_top1_output_class_proba * 100) >= confidence_threshold) {
        da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG classification,%s,conf,%.0f,threshold,%d\r\n", nn_top1_output_class_name, nn_top1_output_class_proba * 100, confidence_threshold);
    }

    // Check for commands over UART
//    char uart_buffer[100];
//    if (HAL_UART_Receive(&huart2, (uint8_t *)uart_buffer, sizeof(uart_buffer), HAL_MAX_DELAY) == HAL_OK) {
//        process_command(uart_buffer);
//    }
	HAL_Delay(2000);
//iotc-stop

    Display_NetworkOutput(&pp_output, ts[1] - ts[0]);
    /* Discard nn_out region (used by pp_input and pp_outputs variables) to avoid Dcache evictions during nn inference */
    for (int i = 0; i < number_output; i++)
    {
      float32_t *tmp = nn_out[i];
      SCB_InvalidateDCache_by_Addr(tmp, nn_out_len[i]);
    }
  }
}

// iotc-start
void process_threshold_command(const char *parameters) {
    int new_threshold = atoi(parameters); // Convert parameter to an integer

    // Validate the threshold range (0 to 100)
    if (new_threshold >= 0 && new_threshold <= 100) {
        confidence_threshold = new_threshold; // Update global confidence threshold
        printf("Confidence threshold updated to: %d%%\n", confidence_threshold);
    } else {
        printf("Invalid confidence threshold received: %s\n", parameters);
    }
}
// iotc-stop

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

/**
* @brief Display Neural Network output classification results as well as other performances informations
*
* @param p_postprocess pointer to postprocessing output
* @param inference_ms inference time in ms
*/
static void Display_NetworkOutput(iseg_postprocess_out_t *p_postprocess, uint32_t inference_ms)
{
  iseg_postprocess_outBuffer_t *rois = p_postprocess->pOutBuff;
  uint32_t nb_rois = p_postprocess->nb_detect;
  int ret;

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
  assert(ret == HAL_OK);

  /* Draw post processing result */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */

    int objects_to_send = (nb_rois > MAX_TRACKED_OBJECTS) ? MAX_TRACKED_OBJECTS : nb_rois;

    char telemetry_buffer[TX_BUFFER_SIZE];
    snprintf(telemetry_buffer, TX_BUFFER_SIZE, "AT+NWICMSG detection_batch,[");

    for (int i = 0; i < objects_to_send; i++)
    {
        float confidence = rois[i].confidence * 100;

        uint32_t x0 = (uint32_t)((rois[i].x_center - rois[i].width / 2) * ((float32_t)lcd_bg_area.XSize)) + lcd_bg_area.X0;
        uint32_t y0 = (uint32_t)((rois[i].y_center - rois[i].height / 2) * ((float32_t)lcd_bg_area.YSize));
        uint32_t width = (uint32_t)(rois[i].width * ((float32_t)lcd_bg_area.XSize));
        uint32_t height = (uint32_t)(rois[i].height * ((float32_t)lcd_bg_area.YSize));

        x0 = (x0 < lcd_bg_area.X0 + lcd_bg_area.XSize) ? x0 : (lcd_bg_area.X0 + lcd_bg_area.XSize - 1);
        y0 = (y0 < lcd_bg_area.Y0 + lcd_bg_area.YSize) ? y0 : (lcd_bg_area.Y0 + lcd_bg_area.YSize - 1);
        width = ((x0 + width) < lcd_bg_area.X0 + lcd_bg_area.XSize) ? width : (lcd_bg_area.X0 + lcd_bg_area.XSize - x0 - 1);
        height = ((y0 + height) < lcd_bg_area.Y0 + lcd_bg_area.YSize) ? height : (lcd_bg_area.Y0 + lcd_bg_area.YSize - y0 - 1);

        int object_id = assign_id_to_object(rois[i].class_index, confidence, x0, y0, width, height);
        if (object_id == -1) continue; // Ignore if object couldn't be assigned an ID

        UTIL_LCD_DrawRect(x0, y0, width, height, colors[i % NUMBER_COLORS]);
        UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, classes_table[rois[i].class_index]);

        char object_data[128];
        snprintf(object_data, sizeof(object_data),
                 "{\"id\":%d,\"class\":\"%s\",\"conf\":%.0f,\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}",
                 object_id, classes_table[rois[i].class_index], confidence, x0, y0, width, height);

        strcat(telemetry_buffer, object_data);
        if (i < objects_to_send - 1)
            strcat(telemetry_buffer, ",");
    }

    strcat(telemetry_buffer, "],inference_time,%lu\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)telemetry_buffer, strlen(telemetry_buffer), HAL_MAX_DELAY);

    UTIL_LCD_SetBackColor(0x40000000);
    UTIL_LCDEx_PrintfAt(0, LINE(2), CENTER_MODE, "Objects: %u", nb_rois);
    UTIL_LCDEx_PrintfAt(0, LINE(20), CENTER_MODE, "Inference: %ums", inference_ms);
    UTIL_LCD_SetBackColor(0);

    SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_FRAMEBUFFER_SIZE);
    ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
    assert(ret == HAL_OK);
    lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;

    HAL_Delay(2000);
}

  for (int32_t i = 0; i < nb_rois; i++)
  {
    /* Display mask */
    for (int x = 0; x < AI_YOLOV8_SEG_PP_MASK_SIZE; x++)
    {
      for (int y = 0; y < AI_YOLOV8_SEG_PP_MASK_SIZE; y++)
      {
        if (rois[i].pMask[y * AI_YOLOV8_SEG_PP_MASK_SIZE + x] > 0.1f)
          UTIL_LCD_FillRect((uint32_t) lcd_bg_area.X0 + x * lcd_bg_area.XSize / AI_YOLOV8_SEG_PP_MASK_SIZE,
                            (uint32_t) lcd_bg_area.Y0 + y * lcd_bg_area.YSize / AI_YOLOV8_SEG_PP_MASK_SIZE,
                            (uint32_t) lcd_bg_area.XSize / AI_YOLOV8_SEG_PP_MASK_SIZE + 1,
                            (uint32_t) lcd_bg_area.YSize / AI_YOLOV8_SEG_PP_MASK_SIZE + 1,
                            colors[i % NUMBER_COLORS] & 0x40ffffff);
      }
    }
  }
  for (int32_t i = 0; i < nb_rois; i++)
  {
    /* Confidence Score */
    float confidence = rois[i].confidence * 100; // Convert to percentage

    /* Get Bounding Box Coordinates */
    uint32_t x0 = (uint32_t)((rois[i].x_center - rois[i].width / 2) * ((float32_t)lcd_bg_area.XSize)) + lcd_bg_area.X0;
    uint32_t y0 = (uint32_t)((rois[i].y_center - rois[i].height / 2) * ((float32_t)lcd_bg_area.YSize));
    uint32_t width = (uint32_t)(rois[i].width * ((float32_t)lcd_bg_area.XSize));
    uint32_t height = (uint32_t)(rois[i].height * ((float32_t)lcd_bg_area.YSize));

    /* Display box */
    uint32_t x0 = (uint32_t) ((rois[i].x_center - rois[i].width / 2) * ((float32_t) lcd_bg_area.XSize)) + lcd_bg_area.X0;
    uint32_t y0 = (uint32_t) ((rois[i].y_center - rois[i].height / 2) * ((float32_t) lcd_bg_area.YSize));
    uint32_t width = (uint32_t) (rois[i].width * ((float32_t) lcd_bg_area.XSize));
    uint32_t height = (uint32_t) (rois[i].height * ((float32_t) lcd_bg_area.YSize));
    /* Draw boxes without going outside of the image */
    x0 = x0 < lcd_bg_area.X0 + lcd_bg_area.XSize ? x0 : lcd_bg_area.X0 + lcd_bg_area.XSize - 1;
    y0 = y0 < lcd_bg_area.Y0 + lcd_bg_area.YSize ? y0 : lcd_bg_area.Y0 + lcd_bg_area.YSize  - 1;
    width = ((x0 + width) < lcd_bg_area.X0 + lcd_bg_area.XSize) ? width : (lcd_bg_area.X0 + lcd_bg_area.XSize - x0 - 1);
    height = ((y0 + height) < lcd_bg_area.Y0 + lcd_bg_area.YSize) ? height : (lcd_bg_area.Y0 + lcd_bg_area.YSize - y0 - 1);
    UTIL_LCD_DrawRect(x0, y0, width, height, colors[i % NUMBER_COLORS]);
    UTIL_LCDEx_PrintfAt(x0, y0, LEFT_MODE, classes_table[rois[i].class_index]);
  }

  UTIL_LCD_SetBackColor(0x40000000);
  UTIL_LCDEx_PrintfAt(0, LINE(2), CENTER_MODE, "Objects %u", nb_rois);
  UTIL_LCDEx_PrintfAt(0, LINE(20), CENTER_MODE, "Inference: %ums", inference_ms);
  UTIL_LCD_SetBackColor(0);

  Display_WelcomeScreen();

  SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_FRAMEBUFFER_SIZE);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
  assert(ret == HAL_OK);
  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

// iotc-start
static void send_iotconnect_data(int class_index, float confidence, int x, int y, int width, int height, uint32_t inference_ms)
{
    char tx_buffer[TX_BUFFER_SIZE];

    // Convert to class name
    const char *class_name = classes_table[class_index];

    // Format message for IoTConnect
    snprintf(tx_buffer, TX_BUFFER_SIZE,
             "AT+NWICMSG detection,class,%s,conf,%.0f,bbox_x,%d,bbox_y,%d,bbox_w,%d,bbox_h,%d,inference_time,%lu\r\n",
             class_name, confidence, x, y, width, height, inference_ms);

    // Send over UART
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}
//iotc - stop

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
    UTIL_LCDEx_PrintfAt(0, LINE(16), CENTER_MODE, "Instance segmentation");
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