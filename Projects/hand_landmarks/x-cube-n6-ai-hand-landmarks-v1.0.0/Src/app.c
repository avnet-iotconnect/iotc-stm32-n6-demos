/**
  ******************************************************************************
  * @file    app.c
  * @author  MDG Application Team
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "app.h"

#include <stdint.h>
#include <stdio.h>

#include "app_cam.h"
#include "app_config.h"
#include "IPL_resize.h"
#include "app_postprocess.h"
#include "isp_api.h"
#include "ld.h"
#include "ll_aton_runtime.h"
#include "cmw_camera.h"
#include "stm32n6570_discovery_lcd.h"
#include "stm32n6570_discovery.h"
#include "stm32_lcd.h"
#include "stm32_lcd_ex.h"
#include "stm32n6xx_hal.h"
#include "tx_api.h"
#include "utils.h"

#include "da16k_comm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HAS_ROTATION_SUPPORT 0

#define LCD_FG_WIDTH LCD_BG_WIDTH
#define LCD_FG_HEIGHT LCD_BG_HEIGHT

#define CACHE_OP(__op__) do { \
  if (is_cache_enable()) { \
    __op__; \
  } \
} while (0)

#define DBG_INFO 0
#define USE_FILTERED_TS 1

#define BQUEUE_MAX_BUFFERS 2
#define CPU_LOAD_HISTORY_DEPTH 8

#define DISPLAY_BUFFER_NB (DISPLAY_DELAY + 2)

/* palm detector */
#define PD_MAX_HAND_NB 1

#define IOTC_INTERVAL 3000

typedef struct {
  float cx;
  float cy;
  float w;
  float h;
  float rotation;
} roi_t;

typedef struct
{
  uint32_t X0;
  uint32_t Y0;
  uint32_t XSize;
  uint32_t YSize;
} Rectangle_TypeDef;

typedef struct {
  TX_SEMAPHORE free;
  TX_SEMAPHORE ready;
  int buffer_nb;
  uint8_t *buffers[BQUEUE_MAX_BUFFERS];
  int free_idx;
  int ready_idx;
} bqueue_t;

typedef struct {
  uint64_t current_total;
  uint64_t current_thread_total;
  uint64_t prev_total;
  uint64_t prev_thread_total;
  struct {
    uint64_t total;
    uint64_t thread;
    uint32_t tick;
  } history[CPU_LOAD_HISTORY_DEPTH];
} cpuload_info_t;

typedef struct {
  int is_valid;
  pd_pp_box_t pd_hands;
  roi_t roi;
  ld_point_t ld_landmarks[LD_LANDMARK_NB];
} hand_info_t;

typedef struct {
  float nn_period_ms;
  uint32_t pd_ms;
  uint32_t hl_ms;
  uint32_t pp_ms;
  uint32_t disp_ms;
  int is_ld_displayed;
  int is_pd_displayed;
  int pd_hand_nb;
  float pd_max_prob;
  hand_info_t hands[PD_MAX_HAND_NB];
} display_info_t;

typedef struct {
  TX_SEMAPHORE update;
  TX_MUTEX lock;
  display_info_t info;
} display_t;

typedef struct {
  uint32_t nn_in_len;
  float *prob_out;
  uint32_t prob_out_len;
  float *boxes_out;
  uint32_t boxes_out_len;
  pd_model_pp_static_param_t static_param;
  pd_postprocess_out_t pd_out;
} pd_model_info_t;

typedef struct {
  uint8_t *nn_in;
  uint32_t nn_in_len;
  float *prob_out;
  uint32_t prob_out_len;
  float *landmarks_out;
  uint32_t landmarks_out_len;
} hl_model_info_t;

typedef struct {
  Button_TypeDef button_id;
  int prev_state;
  void (*on_click_handler)(void *cb_args);
  void *cb_args;
} button_t;

/* Globals */
/* Lcd Background area */
static Rectangle_TypeDef lcd_bg_area = {
  .X0 = (LCD_DEFAULT_WIDTH - LCD_BG_WIDTH) / 2,
  .Y0 = (LCD_DEFAULT_HEIGHT - LCD_BG_HEIGHT) / 2,
  .XSize = LCD_BG_WIDTH,
  .YSize = LCD_BG_HEIGHT,
};
/* Lcd Foreground area */
static Rectangle_TypeDef lcd_fg_area = {
  .X0 = (LCD_DEFAULT_WIDTH - LCD_FG_WIDTH) / 2,
  .Y0 = (LCD_DEFAULT_HEIGHT - LCD_FG_HEIGHT) / 2,
  .XSize = LCD_FG_WIDTH,
  .YSize = LCD_FG_HEIGHT,
};
/* Lcd Background Buffer */
static uint8_t lcd_bg_buffer[DISPLAY_BUFFER_NB][LCD_BG_WIDTH * LCD_BG_HEIGHT * DISPLAY_BPP] ALIGN_32 IN_PSRAM;
static int lcd_bg_buffer_disp_idx = 1;
static int lcd_bg_buffer_capt_idx = 0;
/* Lcd Foreground Buffer */
static uint8_t lcd_fg_buffer[2][LCD_FG_WIDTH * LCD_FG_HEIGHT* 2] ALIGN_32 IN_PSRAM;
static int lcd_fg_buffer_rd_idx;
static display_t disp = {
  .info.is_ld_displayed = 1,
  .info.is_pd_displayed = 0,
};
static cpuload_info_t cpu_load;

/* model */
 /* palm detector */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(palm_detector);
static roi_t rois[PD_MAX_HAND_NB];
 /* hand landmark */
LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(hand_landmark);
static ld_point_t ld_landmarks[PD_MAX_HAND_NB][LD_LANDMARK_NB];
static uint32_t frame_event_nb;
static volatile uint32_t frame_event_nb_for_resize;

 /* nn input buffers */
static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] ALIGN_32 IN_PSRAM;
static bqueue_t nn_input_queue;

 /* threads */
  /* nn thread */
static TX_THREAD nn_thread;
static uint8_t nn_tread_stack[4096];
  /* display thread */
static TX_THREAD dp_thread;
static uint8_t dp_tread_stack[4096];
  /* isp thread */
static TX_THREAD isp_thread;
static uint8_t isp_tread_stack[4096];
static TX_SEMAPHORE isp_sem;


//IOTCONNECT
extern da16k_err_t da16k_at_send_formatted_raw_no_crlf(const char *format, ...);
static float iotc_cpuload = 0.0;
static float iotc_fps = 0.0;
static uint32_t last_send_time = 0;

static int is_cache_enable()
{
#if defined(USE_DCACHE)
  return 1;
#else
  return 0;
#endif
}

static float pd_normalize_angle(float angle)
{
  return angle - 2 * M_PI * floorf((angle - (-M_PI)) / (2 * M_PI));
}

/* Without rotation support allow limited amount of angles */
#if HAS_ROTATION_SUPPORT == 0
static float pd_cook_rotation(float angle)
{
  if (angle >= (3 * M_PI) / 4)
    angle = M_PI;
  else if (angle >= (1 * M_PI) / 4)
    angle = M_PI / 2;
  else if (angle >= -(1 * M_PI) / 4)
    angle = 0;
  else if (angle >= -(3 * M_PI) / 4)
    angle = -M_PI / 2;
  else
    angle = -M_PI;

  return angle;
}
#else
static float pd_cook_rotation(float angle)
{
  return angle;
}
#endif

static float pd_compute_rotation(pd_pp_box_t *box)
{
  float x0, y0, x1, y1;
  float rotation;

  x0 = box->pKps[0].x;
  y0 = box->pKps[0].y;
  x1 = box->pKps[2].x;
  y1 = box->pKps[2].y;

  rotation = M_PI * 0.5 - atan2f(-(y1 - y0), x1 - x0);

  return pd_cook_rotation(pd_normalize_angle(rotation));
}

static void cvt_pd_coord_to_screen_coord(pd_pp_box_t *box)
{
  int i;

  /* This is not a typo. Since screen aspect ratio was conserved. We really want to use LCD_BG_WIDTH for
   * y positions.
   */

  box->x_center *= LCD_BG_WIDTH;
  box->y_center *= LCD_BG_WIDTH;
  box->width *= LCD_BG_WIDTH;
  box->height *= LCD_BG_WIDTH;
  for (i = 0; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++) {
    box->pKps[i].x *= LCD_BG_WIDTH;
    box->pKps[i].y *= LCD_BG_WIDTH;
  }
}

static void roi_shift_and_scale(roi_t *roi, float shift_x, float shift_y, float scale_x, float scale_y)
{
  float long_side;
  float sx, sy;

  sx = (roi->w * shift_x * cos(roi->rotation) - roi->h * shift_y * sin(roi->rotation));
  sy = (roi->w * shift_x * sin(roi->rotation) + roi->h * shift_y * cos(roi->rotation));

  roi->cx += sx;
  roi->cy += sy;

  long_side = MAX(roi->w, roi->h);
  roi->w = long_side;
  roi->h = long_side;

  roi->w *= scale_x;
  roi->h *= scale_y;
}

static void pd_box_to_roi(pd_pp_box_t *box,  roi_t *roi)
{
  const float shift_x = 0;
  const float shift_y = -0.5;
  const float scale = 2.6;

  roi->cx = box->x_center;
  roi->cy = box->y_center;
  roi->w = box->width;
  roi->h = box->height;
  roi->rotation = pd_compute_rotation(box);

  roi_shift_and_scale(roi, shift_x, shift_y, scale, scale);

#if HAS_ROTATION_SUPPORT == 0
  /* In that case we can cancel rotation. This ensure corners are corrected oriented */
  roi->rotation = 0;
#endif
}

static void copy_pd_box(pd_pp_box_t *dst, pd_pp_box_t *src)
{
  int i;

  dst->prob = src->prob;
  dst->x_center = src->x_center;
  dst->y_center = src->y_center;
  dst->width = src->width;
  dst->height = src->height;
  for (i = 0 ; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++)
    dst->pKps[i] = src->pKps[i];
}

static void button_init(button_t *b, Button_TypeDef id, void (*on_click_handler)(void *), void *cb_args)
{
  int ret;

  ret = BSP_PB_Init(id, BUTTON_MODE_GPIO);
  assert(ret == BSP_ERROR_NONE);

  b->button_id = id;
  b->on_click_handler = on_click_handler;
  b->prev_state = 0;
  b->cb_args = cb_args;
}

static void button_process(button_t *b)
{
  int state = BSP_PB_GetState(b->button_id);

  if (state != b->prev_state && state && b->on_click_handler)
    b->on_click_handler(b->cb_args);

  b->prev_state = state;
}

static void cpuload_init(cpuload_info_t *cpu_load)
{
  memset(cpu_load, 0, sizeof(cpuload_info_t));
}

static void cpuload_update(cpuload_info_t *cpu_load)
{
  EXECUTION_TIME thread_total;
  EXECUTION_TIME isr;
  EXECUTION_TIME idle;
  int i;

  cpu_load->history[1] = cpu_load->history[0];

  _tx_execution_thread_total_time_get(&thread_total);
  _tx_execution_isr_time_get(&isr);
  _tx_execution_idle_time_get(&idle);

  cpu_load->history[0].total = thread_total + isr + idle;
  cpu_load->history[0].thread = thread_total;
  cpu_load->history[0].tick = HAL_GetTick();

  if (cpu_load->history[1].tick - cpu_load->history[2].tick < 1000)
    return ;

  for (i = 0; i < CPU_LOAD_HISTORY_DEPTH - 2; i++)
    cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i] = cpu_load->history[CPU_LOAD_HISTORY_DEPTH - 1 - i - 1];
}

static void cpuload_get_info(cpuload_info_t *cpu_load, float *cpu_load_last, float *cpu_load_last_second,
                             float *cpu_load_last_five_seconds)
{
  if (cpu_load_last)
    *cpu_load_last = 100.0 * (cpu_load->history[0].thread - cpu_load->history[1].thread) /
                     (cpu_load->history[0].total - cpu_load->history[1].total);
  if (cpu_load_last_second)
    *cpu_load_last_second = 100.0 * (cpu_load->history[2].thread - cpu_load->history[3].thread) /
                     (cpu_load->history[2].total - cpu_load->history[3].total);
  if (cpu_load_last_five_seconds)
    *cpu_load_last_five_seconds = 100.0 * (cpu_load->history[2].thread - cpu_load->history[7].thread) /
                     (cpu_load->history[2].total - cpu_load->history[7].total);
}

static int bqueue_init(bqueue_t *bq, int buffer_nb, uint8_t **buffers)
{
  int ret;
  int i;

  if (buffer_nb > BQUEUE_MAX_BUFFERS)
    return -1;

  ret = tx_semaphore_create(&bq->free, NULL, buffer_nb);
  if (ret)
    goto free_sem_error;
  ret = tx_semaphore_create(&bq->ready, NULL, 0);
  if (ret)
    goto ready_sem_error;

  bq->buffer_nb = buffer_nb;
  for (i = 0; i < buffer_nb; i++) {
    assert(buffers[i]);
    bq->buffers[i] = buffers[i];
  }
  bq->free_idx = 0;
  bq->ready_idx = 0;

  return 0;

ready_sem_error:
  tx_semaphore_delete(&bq->free);
free_sem_error:
  return -1;
}

static uint8_t *bqueue_get_free(bqueue_t *bq, int is_blocking)
{
  uint8_t *res;
  int ret;

  ret = tx_semaphore_get(&bq->free, is_blocking ? TX_WAIT_FOREVER : TX_NO_WAIT);
  if (ret == TX_NO_INSTANCE)
    return NULL;
  assert(ret == 0);

  res = bq->buffers[bq->free_idx];
  bq->free_idx = (bq->free_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_free(bqueue_t *bq)
{
  int ret;

  ret = tx_semaphore_put(&bq->free);
  assert(ret == 0);
}

static uint8_t *bqueue_get_ready(bqueue_t *bq)
{
  uint8_t *res;
  int ret;

  ret = tx_semaphore_get(&bq->ready, TX_WAIT_FOREVER);
  assert(ret == 0);

  res = bq->buffers[bq->ready_idx];
  bq->ready_idx = (bq->ready_idx + 1) % bq->buffer_nb;

  return res;
}

static void bqueue_put_ready(bqueue_t *bq)
{
  int ret;

  ret = tx_semaphore_put(&bq->ready);
  assert(ret == 0);
}

static void app_main_pipe_frame_event()
{
  int next_disp_idx = (lcd_bg_buffer_disp_idx + 1) % DISPLAY_BUFFER_NB;
  int next_capt_idx = (lcd_bg_buffer_capt_idx + 1) % DISPLAY_BUFFER_NB;
  int ret;

  ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE1,
                                         DCMIPP_MEMORY_ADDRESS_0, (uint32_t) lcd_bg_buffer[next_capt_idx]);
  assert(ret == HAL_OK);

  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_bg_buffer[next_disp_idx], LTDC_LAYER_1);
  assert(ret == HAL_OK);
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_1);
  assert(ret == HAL_OK);
  lcd_bg_buffer_disp_idx = next_disp_idx;
  lcd_bg_buffer_capt_idx = next_capt_idx;

  frame_event_nb++;
}


static void app_ancillary_pipe_frame_event()
{
  uint8_t *next_buffer;
  int ret;

  next_buffer = bqueue_get_free(&nn_input_queue, 0);
  if (next_buffer) {
    ret = HAL_DCMIPP_PIPE_SetMemoryAddress(CMW_CAMERA_GetDCMIPPHandle(), DCMIPP_PIPE2,
                                           DCMIPP_MEMORY_ADDRESS_0, (uint32_t) next_buffer);
    assert(ret == HAL_OK);
    /* minus 1 since app_main_pipe_frame_event occur before app_ancillary_pipe_frame_event() */
    frame_event_nb_for_resize = frame_event_nb - 1;
    bqueue_put_ready(&nn_input_queue);
  }
}

static void app_main_pipe_vsync_event()
{
  int ret;

  ret = tx_semaphore_put(&isp_sem);
  assert(ret == 0);
}

static void LCD_init()
{
  BSP_LCD_LayerConfig_t LayerConfig = {0};

  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);

  /* Preview layer Init */
  LayerConfig.X0          = lcd_bg_area.X0;
  LayerConfig.Y0          = lcd_bg_area.Y0;
  LayerConfig.X1          = lcd_bg_area.X0 + lcd_bg_area.XSize;
  LayerConfig.Y1          = lcd_bg_area.Y0 + lcd_bg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_RGB888;
  LayerConfig.Address     = (uint32_t) lcd_bg_buffer[lcd_bg_buffer_disp_idx];

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_1, &LayerConfig);

  LayerConfig.X0 = lcd_fg_area.X0;
  LayerConfig.Y0 = lcd_fg_area.Y0;
  LayerConfig.X1 = lcd_fg_area.X0 + lcd_fg_area.XSize;
  LayerConfig.Y1 = lcd_fg_area.Y0 + lcd_fg_area.YSize;
  LayerConfig.PixelFormat = LCD_PIXEL_FORMAT_ARGB4444;
  LayerConfig.Address = (uint32_t) lcd_fg_buffer[1]; /* External XSPI1 PSRAM */

  BSP_LCD_ConfigLayer(0, LTDC_LAYER_2, &LayerConfig);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);
  UTIL_LCD_SetLayer(LTDC_LAYER_2);
  UTIL_LCD_Clear(0x00000000);
  UTIL_LCD_SetFont(&Font20);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
}

static HAL_StatusTypeDef MX_LTDC_ConfigLayer_Layer1(LTDC_HandleTypeDef *hltdc, MX_LTDC_LayerConfig_t *Config)
{
  LTDC_LayerFlexARGBTypeDef pLayerCfg = {0};

  pLayerCfg.Layer.WindowX0 = Config->X0;
  pLayerCfg.Layer.WindowX1 = Config->X1;
  pLayerCfg.Layer.WindowY0 = Config->Y0;
  pLayerCfg.Layer.WindowY1 = Config->Y1;
  pLayerCfg.Layer.Alpha = LTDC_LxCACR_CONSTA;
  pLayerCfg.Layer.Alpha0 = 0;
  pLayerCfg.Layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.Layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.Layer.ImageWidth = (Config->X1 - Config->X0);
  pLayerCfg.Layer.ImageHeight = (Config->Y1 - Config->Y0);

  /* See ticket 188827 */
  pLayerCfg.FlexARGB.PixelSize = 3;/*LTDC_ARGB_PIXEL_SIZE_3_BYTES;*/
  pLayerCfg.FlexARGB.AlphaPos = 0;
  pLayerCfg.FlexARGB.RedPos = 0;
  pLayerCfg.FlexARGB.GreenPos = 8;
  pLayerCfg.FlexARGB.BluePos = 16;
  pLayerCfg.FlexARGB.AlphaWidth = 0;
  pLayerCfg.FlexARGB.RedWidth = 8;
  pLayerCfg.FlexARGB.GreenWidth = 8;
  pLayerCfg.FlexARGB.BlueWidth = 8;

  pLayerCfg.ARGBAddress = Config->Address;

  return HAL_LTDC_ConfigLayerFlexARGB(hltdc, &pLayerCfg, LTDC_LAYER_1);
}

static HAL_StatusTypeDef MX_LTDC_ConfigLayer_Layer2(LTDC_HandleTypeDef *hltdc, MX_LTDC_LayerConfig_t *Config)
{
  LTDC_LayerCfgTypeDef pLayerCfg ={0};

  pLayerCfg.WindowX0 = Config->X0;
  pLayerCfg.WindowX1 = Config->X1;
  pLayerCfg.WindowY0 = Config->Y0;
  pLayerCfg.WindowY1 = Config->Y1;
  pLayerCfg.PixelFormat = Config->PixelFormat;
  pLayerCfg.Alpha = LTDC_LxCACR_CONSTA;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = Config->Address;
  pLayerCfg.ImageWidth = (Config->X1 - Config->X0);
  pLayerCfg.ImageHeight = (Config->Y1 - Config->Y0);
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;

  return HAL_LTDC_ConfigLayer(hltdc, &pLayerCfg, LTDC_LAYER_2);
}

static int clamp_point(int *x, int *y)
{
  int xi = *x;
  int yi = *y;

  if (*x < 0)
    *x = 0;
  if (*y < 0)
    *y = 0;
  if (*x >= lcd_bg_area.XSize)
    *x = lcd_bg_area.XSize - 1;
  if (*y >= lcd_bg_area.YSize)
    *y = lcd_bg_area.YSize - 1;

  return (xi != *x) || (yi != *y);
}

static int clamp_point_with_margin(int *x, int *y, int margin)
{
  int xi = *x;
  int yi = *y;

  if (*x < margin)
    *x = margin;
  if (*y < margin)
    *y = margin;
  if (*x >= lcd_bg_area.XSize - margin)
    *x = lcd_bg_area.XSize - margin - 1;
  if (*y >= lcd_bg_area.YSize - margin)
    *y = lcd_bg_area.YSize - margin - 1;

  return (xi != *x) || (yi != *y);
}

static void display_pd_hand(pd_pp_box_t *hand)
{
  int xc, yc;
  int x0, y0;
  int x1, y1;
  int w, h;
  int i;

  /* display box around palm */
  xc = (int)hand->x_center;
  yc = (int)hand->y_center;
  w = (int)hand->width;
  h = (int)hand->height;
  x0 = xc - (w + 1) / 2;
  y0 = yc - (h + 1) / 2;
  x1 = xc + (w + 1) / 2;
  y1 = yc + (h + 1) / 2;
  clamp_point(&x0, &y0);
  clamp_point(&x1, &y1);
  UTIL_LCD_DrawRect(x0, y0, x1 - x0, y1 - y0, UTIL_LCD_COLOR_GREEN);

  /* display palm key points */
  for (i = 0; i < 7; i++) {
    uint32_t color = (i != 0 && i != 2) ? UTIL_LCD_COLOR_RED : UTIL_LCD_COLOR_BLUE;

    x0 = (int)hand->pKps[i].x;
    y0 = (int)hand->pKps[i].y;
    clamp_point(&x0, &y0);
    UTIL_LCD_FillCircle(x0, y0, 2, color);
  }
}

static void rotate_point(float pt[2], float rotation)
{
  float x = pt[0];
  float y = pt[1];

  pt[0] = cos(rotation) * x - sin(rotation) * y;
  pt[1] = sin(rotation) * x + cos(rotation) * y;
}

static void roi_to_corners(roi_t *roi, float corners[4][2])
{
  const float corners_init[4][2] = {
    {-roi->w / 2, -roi->h / 2},
    { roi->w / 2, -roi->h / 2},
    { roi->w / 2,  roi->h / 2},
    {-roi->w / 2,  roi->h / 2},
  };
  int i;

  memcpy(corners, corners_init, sizeof(corners_init));
  /* rotate */
  for (i = 0; i < 4; i++)
    rotate_point(corners[i], roi->rotation);

  /* shift */
  for (i = 0; i < 4; i++) {
    corners[i][0] += roi->cx;
    corners[i][1] += roi->cy;
  }
}

static int clamp_corners(float corners_in[4][2], int corners_out[4][2])
{
  int is_clamp = 0;
  int i;

  for (i = 0; i < 4; i++) {
    corners_out[i][0] = (int)corners_in[i][0];
    corners_out[i][1] = (int)corners_in[i][1];
    is_clamp |= clamp_point(&corners_out[i][0], &corners_out[i][1]);
  }

  return is_clamp;
}

static void display_roi(roi_t *roi)
{
  float corners_f[4][2];
  int corners[4][2];
  int is_clamp;
  int i;

  /* compute box corners */
  roi_to_corners(roi, corners_f);

  /* clamp */
  is_clamp = clamp_corners(corners_f, corners);
  if (is_clamp)
    return ;

  /* display */
  for (i = 0; i < 4; i++)
    UTIL_LCD_DrawLine(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                      UTIL_LCD_COLOR_RED);
}

static void decode_ld_landmark(roi_t *roi, ld_point_t *lm, ld_point_t *decoded)
{
  float rotation = roi->rotation;
  float w = roi->w;
  float h = roi->h;

  decoded->x = roi->cx + (lm->x - 0.5) * w * cos(rotation) - (lm->y - 0.5) * h * sin(rotation);
  decoded->y = roi->cy + (lm->x - 0.5) * w * sin(rotation) + (lm->y - 0.5) * h * cos(rotation);
}

static void display_ld_hand(hand_info_t *hand)
{
  const int disk_radius = 2;
  roi_t *roi = &hand->roi;
  int x[LD_LANDMARK_NB];
  int y[LD_LANDMARK_NB];
  int is_clamped[LD_LANDMARK_NB];
  ld_point_t decoded;
  int i;

  for (i = 0; i < LD_LANDMARK_NB; i++) {
    decode_ld_landmark(roi, &hand->ld_landmarks[i], &decoded);
    x[i] = (int)decoded.x;
    y[i] = (int)decoded.y;
    is_clamped[i] = clamp_point_with_margin(&x[i], &y[i], disk_radius);
  }

  for (i = 0; i < LD_LANDMARK_NB; i++) {
    if (is_clamped[i])
      continue;
    UTIL_LCD_FillCircle(x[i], y[i], disk_radius, UTIL_LCD_COLOR_YELLOW);
  }

  for (i = 0; i < LD_BINDING_NB; i++) {
    if (is_clamped[ld_bindings_idx[i][0]] || is_clamped[ld_bindings_idx[i][1]])
      continue;
    UTIL_LCD_DrawLine(x[ld_bindings_idx[i][0]], y[ld_bindings_idx[i][0]],
                      x[ld_bindings_idx[i][1]], y[ld_bindings_idx[i][1]],
                      UTIL_LCD_COLOR_BLACK);
  }
}

void display_hand(display_info_t *info, hand_info_t *hand)
{
  if (info->is_pd_displayed) {
    display_pd_hand(&hand->pd_hands);
    display_roi(&hand->roi);
  }
  if (info->is_ld_displayed)
    display_ld_hand(hand);
}

static void Display_NetworkOutput(display_info_t *info)
{
  float cpu_load_one_second;
  int line_nb = 0;
  float nn_fps;
  int i;

  /* clear previous ui */
  UTIL_LCD_FillRect(lcd_fg_area.X0, lcd_fg_area.Y0, lcd_fg_area.XSize, lcd_fg_area.YSize, 0x00000000); /* Clear previous boxes */

  /* cpu load */
  cpuload_update(&cpu_load);
  cpuload_get_info(&cpu_load, NULL, &cpu_load_one_second, NULL);

  /* draw metrics */
  nn_fps = 1000.0 / info->nn_period_ms;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "Cpu load");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "   %.1f%%", cpu_load_one_second);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Inferences");
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " pd %2ums", info->pd_ms);
  line_nb += 1;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, " hl %2ums", info->hl_ms);
  line_nb += 2;
  UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "  %.1f FPS", nn_fps);
  line_nb += 2;
  if (DBG_INFO) {
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "Display");
    line_nb += 1;
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb), RIGHT_MODE, "   %ums", info->disp_ms);
    line_nb += 1;
  }
  //IOTCONNECT
  iotc_cpuload = cpu_load_one_second;
  iotc_fps = nn_fps;

  /* display palm detector output */
  for (i = 0; i < info->pd_hand_nb; i++) {
    if (info->hands[i].is_valid)
      display_hand(info, &info->hands[i]);
  }

  if (DBG_INFO)
    UTIL_LCDEx_PrintfAt(0, LINE(line_nb),  RIGHT_MODE, "pd : %5.1f %%", info->pd_max_prob * 100);
}

static void palm_detector_init(pd_model_info_t *info)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_palm_detector();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_palm_detector();
  int ret;

  /* model info */
  info->nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  info->prob_out = (float *) LL_Buffer_addr_start(&nn_out_info[0]);
  info->prob_out_len = LL_Buffer_len(&nn_out_info[0]);
  assert(info->prob_out_len == AI_PD_MODEL_PP_TOTAL_DETECTIONS * sizeof(float));
  info->boxes_out = (float *) LL_Buffer_addr_start(&nn_out_info[1]);
  info->boxes_out_len = LL_Buffer_len(&nn_out_info[1]);
  assert(info->boxes_out_len == AI_PD_MODEL_PP_TOTAL_DETECTIONS * sizeof(float) * 18);

  /* post processor info */
  ret = app_postprocess_init(&info->static_param);
  assert(ret == AI_PD_POSTPROCESS_ERROR_NO);
}

static int palm_detector_run(uint8_t *buffer, pd_model_info_t *info, uint32_t *pd_exec_time)
{
  uint32_t start_ts;
  int hand_nb;
  int ret;
  int i;

  start_ts = HAL_GetTick();
  /* Note that we don't need to clean/invalidate those input buffers since they are only access in hardware */
  ret = LL_ATON_Set_User_Input_Buffer_palm_detector(0, buffer, info->nn_in_len);
  assert(ret == LL_ATON_User_IO_NOERROR);

  LL_ATON_RT_Main(&NN_Instance_palm_detector);

  ret = app_postprocess_run((void * []){info->prob_out, info->boxes_out}, 2, &info->pd_out, &info->static_param);
  assert(ret == AI_PD_POSTPROCESS_ERROR_NO);
  hand_nb = MIN(info->pd_out.box_nb, PD_MAX_HAND_NB);

  for (i = 0; i < hand_nb; i++) {
    cvt_pd_coord_to_screen_coord(&info->pd_out.pOutData[i]);
    pd_box_to_roi(&info->pd_out.pOutData[i], &rois[i]);
  }

  /* Discard nn_out region (used by pp_outputs variables) to avoid Dcache evictions during nn inference */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->prob_out, info->prob_out_len));
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->boxes_out, info->boxes_out_len));

  *pd_exec_time = HAL_GetTick() - start_ts;

  return hand_nb;
}

static void hand_landmark_init(hl_model_info_t *info)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_hand_landmark();
  const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_hand_landmark();

  info->nn_in = LL_Buffer_addr_start(&nn_in_info[0]);
  info->nn_in_len = LL_Buffer_len(&nn_in_info[0]);
  info->prob_out = (float *) LL_Buffer_addr_start(&nn_out_info[2]);
  info->prob_out_len = LL_Buffer_len(&nn_out_info[2]);
  assert(info->prob_out_len == sizeof(float));
  info->landmarks_out = (float *) LL_Buffer_addr_start(&nn_out_info[3]);
  info->landmarks_out_len = LL_Buffer_len(&nn_out_info[3]);
  assert(info->landmarks_out_len == sizeof(float) * 63);
}

#if HAS_ROTATION_SUPPORT == 0
static int hand_landmark_prepare_input(uint8_t *buffer, roi_t *roi, hl_model_info_t *info)
{
  float corners_f[4][2];
  int corners[4][2];
  uint8_t* out_data;
  size_t height_out;
  uint8_t *in_data;
  size_t height_in;
  size_t width_out;
  size_t width_in;
  int is_clamped;

  /* defaults when no clamping occurs */
  out_data = info->nn_in;
  width_out = LD_WIDTH;
  height_out = LD_HEIGHT;

  roi_to_corners(roi, corners_f);
  is_clamped = clamp_corners(corners_f, corners);

  /* If clamp perform a partial resize */
  if (is_clamped) {
    int offset_x;
    int offset_y;

    /* clear target memory since resize will partially write it */
    memset(info->nn_in, 0, info->nn_in_len);

    /* compute start address of output buffer */
    offset_x = (int)(((corners[0][0] - corners_f[0][0]) * LD_WIDTH) / (corners_f[2][0] - corners_f[0][0]));
    offset_y = (int)(((corners[0][1] - corners_f[0][1]) * LD_HEIGHT) / (corners_f[2][1] - corners_f[0][1]));
    out_data += offset_y * (int)LD_WIDTH * DISPLAY_BPP + offset_x * DISPLAY_BPP;

    /* compute output width and height */
    width_out = (int)((corners[2][0] - corners[0][0]) / (corners_f[2][0] - corners_f[0][0]) * LD_WIDTH);
    height_out = (int)((corners[2][1] - corners[0][1]) / (corners_f[2][1] - corners_f[0][1]) * LD_HEIGHT);

    assert(width_out > 0);
    assert(height_out > 0);
    {
      uint8_t* out_data_end;

      out_data_end = out_data + (int)LD_WIDTH * DISPLAY_BPP * (height_out - 1) + DISPLAY_BPP * width_out - 1;

      assert(out_data_end >= info->nn_in);
      assert(out_data_end < info->nn_in + info->nn_in_len);
    }
  }

  in_data = buffer + corners[0][1] * LCD_BG_WIDTH * DISPLAY_BPP + corners[0][0]* DISPLAY_BPP;
  width_in = corners[2][0] - corners[0][0];
  height_in = corners[2][1] - corners[0][1];

  assert(width_in > 0);
  assert(height_in > 0);
  {
    uint8_t* in_data_end;

    in_data_end = in_data + LCD_BG_WIDTH * DISPLAY_BPP * (height_in - 1) + DISPLAY_BPP * width_in - 1;

    assert(in_data_end >= buffer);
    assert(in_data_end < buffer + LCD_BG_WIDTH * LCD_BG_HEIGHT * DISPLAY_BPP);
  }

  IPL_resize_bilinear_iu8ou8_with_strides_RGB(in_data, out_data, LCD_BG_WIDTH * DISPLAY_BPP, LD_WIDTH * DISPLAY_BPP,
                                              width_in, height_in, width_out, height_out);

  return 0;
}
#else
#error "implement hand_landmark_prepare_input with ROTATION support"
#endif

static int hand_landmark_run(uint8_t *buffer, hl_model_info_t *info, roi_t *roi,
                             ld_point_t ld_landmarks[LD_LANDMARK_NB])
{
  int is_clamped;
  int is_valid;

  is_clamped = hand_landmark_prepare_input(buffer, roi, info);
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(info->nn_in, info->nn_in_len));
  if (is_clamped)
    return 0;

  LL_ATON_RT_Main(&NN_Instance_hand_landmark);

  is_valid = ld_post_process(info->prob_out, info->landmarks_out, ld_landmarks);

  /* Discard nn_out region (used by pp_input and pp_outputs variables) to avoid Dcache evictions during nn inference */
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->prob_out, info->prob_out_len));
  CACHE_OP(SCB_InvalidateDCache_by_Addr(info->landmarks_out, info->landmarks_out_len));

  return is_valid;
}

static float ld_compute_rotation(ld_point_t lm[LD_LANDMARK_NB])
{
  float x0, y0, x1, y1;
  float rotation;

  x0 = lm[0].x;
  y0 = lm[0].y;
  x1 = lm[9].x;
  y1 = lm[9].y;

  rotation = M_PI * 0.5 - atan2f(-(y1 - y0), x1 - x0);

  return pd_cook_rotation(pd_normalize_angle(rotation));
}

static void ld_to_roi(ld_point_t lm[LD_LANDMARK_NB], roi_t *roi, pd_pp_box_t *next_pd)
{
  const int pd_to_ld_idx[AI_PD_MODEL_PP_NB_KEYPOINTS] = {0, 5, 9, 13, 17, 1, 2};
  const int indices[] = {0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18};
  float max_x, max_y, min_x, min_y;
  int i;

  max_x = max_y = -10000;
  min_x = min_y =  10000;

  roi->rotation = ld_compute_rotation(lm);

  for (i = 0; i < ARRAY_NB(indices); i++) {
    max_x = MAX(max_x, lm[indices[i]].x);
    max_y = MAX(max_y, lm[indices[i]].y);
    min_x = MIN(min_x, lm[indices[i]].x);
    min_y = MIN(min_y, lm[indices[i]].y);
  }

  roi->cx = (max_x + min_x) / 2;
  roi->cy = (max_y + min_y) / 2;
  roi->w = (max_x - min_x);
  roi->h = (max_y - min_y);

  next_pd->x_center = roi->cx;
  next_pd->y_center = roi->cy;
  next_pd->width = roi->w;
  next_pd->height = roi->h;
  for (i = 0; i < AI_PD_MODEL_PP_NB_KEYPOINTS; i++) {
    next_pd->pKps[i].x = lm[pd_to_ld_idx[i]].x;
    next_pd->pKps[i].y = lm[pd_to_ld_idx[i]].y;
  }
}

static void compute_next_roi(roi_t *src, ld_point_t lm_in[LD_LANDMARK_NB], roi_t *next, pd_pp_box_t *next_pd)
{
  const float shift_x = 0;
  const float shift_y = -0.1;
  const float scale = 2.0;
  ld_point_t lm[LD_LANDMARK_NB];
  roi_t roi;
  int i;

  for (i = 0; i < LD_LANDMARK_NB; i++)
    decode_ld_landmark(src, &lm_in[i], &lm[i]);

  ld_to_roi(lm, &roi, next_pd);
  roi_shift_and_scale(&roi, shift_x, shift_y, scale, scale);

#if HAS_ROTATION_SUPPORT == 0
  /* In that case we can cancel rotation. This ensure corners are corrected oriented */
  roi.rotation = 0;
#endif

  *next = roi;
}

static void nn_thread_fct(ULONG arg)
{
  float nn_period_filtered_ms = 0;
  float pd_filtered_ms = 0;
  float ld_filtered_ms = 0;
  hl_model_info_t hl_info;
  pd_model_info_t pd_info;
  uint32_t nn_period_ms;
  uint32_t nn_period[2];
  uint8_t *nn_pipe_dst;
  pd_pp_point_t box_next_keypoints[AI_PD_MODEL_PP_NB_KEYPOINTS];
  pd_pp_box_t box_next;
  int is_tracking = 0;
  roi_t roi_next;
  uint32_t pd_ms;
  uint32_t hl_ms;
  int j;

  /* Current tracking algo only support single hand */
  assert(PD_MAX_HAND_NB == 1);

  /* setup models buffer info */
  palm_detector_init(&pd_info);
  box_next.pKps = box_next_keypoints;
  hand_landmark_init(&hl_info);

  /*** App Loop ***************************************************************/
  nn_period[1] = HAL_GetTick();
  nn_pipe_dst = bqueue_get_free(&nn_input_queue, 0);
  assert(nn_pipe_dst);
  CAM_NNPipe_Start(nn_pipe_dst, CMW_MODE_CONTINUOUS);
  while (1)
  {
    uint8_t *capture_buffer;
    int idx_for_resize;

    nn_period[0] = nn_period[1];
    nn_period[1] = HAL_GetTick();
    nn_period_ms = nn_period[1] - nn_period[0];
    nn_period_filtered_ms = USE_FILTERED_TS ? (15 * nn_period_filtered_ms + nn_period_ms) / 16 : nn_period_ms;

    capture_buffer = bqueue_get_ready(&nn_input_queue);
    assert(capture_buffer);
    idx_for_resize = frame_event_nb_for_resize % DISPLAY_BUFFER_NB;

    /* Only start palm detector when not tracking hand */
    if (!is_tracking) {
      is_tracking = palm_detector_run(capture_buffer, &pd_info, &pd_ms);
      box_next.prob = pd_info.pd_out.pOutData[0].prob;
    } else {
      rois[0] = roi_next;
      copy_pd_box(&pd_info.pd_out.pOutData[0], &box_next);
      pd_ms = 0;
    }
    pd_filtered_ms = USE_FILTERED_TS ? (7 * pd_filtered_ms + pd_ms) / 8 : pd_ms;
    bqueue_put_free(&nn_input_queue);

    /* then run hand landmark detector if needed */
    if (is_tracking) {
      hl_ms = HAL_GetTick();
      is_tracking = hand_landmark_run(lcd_bg_buffer[idx_for_resize], &hl_info, &rois[0], ld_landmarks[0]);
      CACHE_OP(SCB_InvalidateDCache_by_Addr(lcd_bg_buffer[idx_for_resize], sizeof(lcd_bg_buffer[idx_for_resize])));
      if (is_tracking)
        compute_next_roi(&rois[0], ld_landmarks[0], &roi_next, &box_next);
      hl_ms = HAL_GetTick() - hl_ms;
    } else {
      hl_ms = 0;
    }
    ld_filtered_ms = USE_FILTERED_TS ? (7 * ld_filtered_ms + hl_ms) / 8 : hl_ms;

    /* update display stats */
    tx_mutex_get(&disp.lock, TX_WAIT_FOREVER);
    disp.info.pd_ms = is_tracking ? 0 : (int)pd_filtered_ms;
    disp.info.hl_ms = is_tracking ? (int)ld_filtered_ms : 0;
    disp.info.nn_period_ms = nn_period_filtered_ms;
    disp.info.pd_hand_nb = is_tracking;
    disp.info.pd_max_prob = pd_info.pd_out.pOutData[0].prob;
    disp.info.hands[0].is_valid = is_tracking;
    copy_pd_box(&disp.info.hands[0].pd_hands, &pd_info.pd_out.pOutData[0]);
    disp.info.hands[0].roi = rois[0];
    for (j = 0; j < LD_LANDMARK_NB; j++)
      disp.info.hands[0].ld_landmarks[j] = ld_landmarks[0][j];
    tx_mutex_put(&disp.lock);

    tx_semaphore_ceiling_put(&disp.update, 1);
  }
}

static void dp_update_drawing_area()
{
  int ret;

  __disable_irq();
  ret = HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, (uint32_t) lcd_fg_buffer[lcd_fg_buffer_rd_idx], LTDC_LAYER_2);
  assert(ret == HAL_OK);
  __enable_irq();
}

static void dp_commit_drawing_area()
{
  int ret;

  __disable_irq();
  ret = HAL_LTDC_ReloadLayer(&hlcd_ltdc, LTDC_RELOAD_VERTICAL_BLANKING, LTDC_LAYER_2);
  assert(ret == HAL_OK);
  __enable_irq();
  lcd_fg_buffer_rd_idx = 1 - lcd_fg_buffer_rd_idx;
}

static void on_ld_toggle_button_click(void *args)
{
  display_t *disp = (display_t *) args;

  tx_mutex_get(&disp->lock, TX_WAIT_FOREVER);
  disp->info.is_ld_displayed = !disp->info.is_ld_displayed;
  tx_mutex_put(&disp->lock);
}

static void on_pd_toggle_button_click(void *args)
{
  display_t *disp = (display_t *) args;

  tx_mutex_get(&disp->lock, TX_WAIT_FOREVER);
  disp->info.is_pd_displayed = !disp->info.is_pd_displayed;
  tx_mutex_put(&disp->lock);
}

static void dp_thread_fct(ULONG arg)
{
  button_t ld_toggle_button;
  button_t hd_toggle_button;
  uint32_t disp_ms = 0;
  display_info_t info;
  uint32_t ts;
  int ret;

  button_init(&ld_toggle_button, BUTTON_USER1, on_ld_toggle_button_click, &disp);
  button_init(&hd_toggle_button, BUTTON_TAMP, on_pd_toggle_button_click, &disp);
  while (1)
  {
	iotc_cpuload = 0;
	iotc_fps = 0;

    ret = tx_semaphore_get(&disp.update, TX_WAIT_FOREVER);
    assert(ret == 0);

    button_process(&ld_toggle_button);
    button_process(&hd_toggle_button);

    tx_mutex_get(&disp.lock, TX_WAIT_FOREVER);
    info = disp.info;
    tx_mutex_put(&disp.lock);
    info.disp_ms = disp_ms;

    ts = HAL_GetTick();
    dp_update_drawing_area();
    Display_NetworkOutput(&info);
    SCB_CleanDCache_by_Addr(lcd_fg_buffer[lcd_fg_buffer_rd_idx], LCD_FG_WIDTH * LCD_FG_HEIGHT* 2);
    dp_commit_drawing_area();
    disp_ms = HAL_GetTick() - ts;

    //IOTCONNECT
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_send_time) < IOTC_INTERVAL) {
      continue; //send messages to iotconnect every IOTC_INTERVAL
    }
    last_send_time = current_time;

    /* box area around palm */
    int x_center = 0;
    int y_center = 0;
    int w_data = 0;
    int h_data = 0;

    if (info.pd_hand_nb > 0) {
      x_center = (int)info.hands[0].pd_hands.x_center;
      y_center = (int)info.hands[0].pd_hands.y_center;
      w_data = (int)info.hands[0].pd_hands.width;
      h_data = (int)info.hands[0].pd_hands.height;
      clamp_point(&x_center, &y_center);
    } else {
      iotc_cpuload = 0;
      iotc_fps = 0;
    }
    printf("Sending the message to IOTCONNECT...\r\n");
	da16k_at_send_formatted_raw_no_crlf("AT+NWICMSG x_center,%d,y_center,%d,width,%d,height,%d,cpu_load,%f,fps,%f\r\n",x_center, y_center, w_data, h_data, iotc_cpuload, iotc_fps);

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

static void isp_thread_fct(ULONG arg)
{
  int ret;

  while (1) {
    ret = tx_semaphore_get(&isp_sem, TX_WAIT_FOREVER);
    assert(ret == 0);

    CAM_IspUpdate();
  }
}

void app_run()
{
  const UINT isp_priority = TX_MAX_PRIORITIES / 2 - 2;
  const UINT dp_priority = TX_MAX_PRIORITIES / 2 + 2;
  const UINT nn_priority = TX_MAX_PRIORITIES / 2 - 1;
  const ULONG time_slice = 10;
  int ret;

  printf("Init application\n");
  /* Enable DWT so DWT_CYCCNT works when debugger not attached */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* screen init */
  memset(lcd_bg_buffer, 0, sizeof(lcd_bg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_bg_buffer, sizeof(lcd_bg_buffer)));
  memset(lcd_fg_buffer, 0, sizeof(lcd_fg_buffer));
  CACHE_OP(SCB_CleanInvalidateDCache_by_Addr(lcd_fg_buffer, sizeof(lcd_fg_buffer)));
  LCD_init();

  /* create buffer queues */
  ret = bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
  assert(ret == 0);

  cpuload_init(&cpu_load);

  /*** Camera Init ************************************************************/  
  CAM_Init();

  /* da16k module init */
  da16k_cfg_t cfg = {0};
  da16k_init(&cfg);

  /* sems + mutex init */
  ret = tx_semaphore_create(&isp_sem, NULL, 0);
  assert(ret == 0);
  ret = tx_semaphore_create(&disp.update, NULL, 0);
  assert(ret == 0);
  ret= tx_mutex_create(&disp.lock, NULL, TX_INHERIT);
  assert(ret == 0);

  /* Start LCD Display camera pipe stream */
  CAM_DisplayPipe_Start(lcd_bg_buffer[0], CMW_MODE_CONTINUOUS);

  /* threads init */
  ret = tx_thread_create(&nn_thread, "nn", nn_thread_fct, 0, nn_tread_stack,
                         sizeof(nn_tread_stack), nn_priority, nn_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
  ret = tx_thread_create(&dp_thread, "dp", dp_thread_fct, 0, dp_tread_stack,
                         sizeof(dp_tread_stack), dp_priority, dp_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
  ret = tx_thread_create(&isp_thread, "isp", isp_thread_fct, 0, isp_tread_stack,
                         sizeof(isp_tread_stack), isp_priority, isp_priority, time_slice, TX_AUTO_START);
  assert(ret == TX_SUCCESS);
}

int CMW_CAMERA_PIPE_FrameEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_frame_event();
  else if (pipe == DCMIPP_PIPE2)
    app_ancillary_pipe_frame_event();

  return HAL_OK;
}

int CMW_CAMERA_PIPE_VsyncEventCallback(uint32_t pipe)
{
  if (pipe == DCMIPP_PIPE1)
    app_main_pipe_vsync_event();

  return HAL_OK;
}

/* Override to allow usage of flexible ARGB for layer 1 */
HAL_StatusTypeDef MX_LTDC_ConfigLayer(LTDC_HandleTypeDef *hltdc, uint32_t LayerIndex, MX_LTDC_LayerConfig_t *Config)
{
  if (LayerIndex == LTDC_LAYER_1)
    return MX_LTDC_ConfigLayer_Layer1(hltdc, Config);

  if (LayerIndex == LTDC_LAYER_2)
    return MX_LTDC_ConfigLayer_Layer2(hltdc, Config);

  assert(0);

  return HAL_ERROR;
}
