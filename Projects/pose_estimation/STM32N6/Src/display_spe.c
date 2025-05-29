 /**
 ******************************************************************************
 * @file    display_spe.c
 * @author  GPM Application Team
 *
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

#include "display_spe.h"
#include "app_config.h"
#include "main.h"
#if POSTPROCESS_TYPE == POSTPROCESS_SPE_MOVENET_UF
  #if AI_POSE_PP_POSE_KEYPOINTS_NB == 17
    #include "display_keypoints_17.h"
  #elif AI_POSE_PP_POSE_KEYPOINTS_NB == 13
    #include "display_keypoints_13.h"
  #else
    #error "Unsupported number of keypoints"
  #endif
#endif
#include "utils.h"
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

static int (*clamp_point)(int *x, int *y);
static void (*convert_length)(float32_t wi, float32_t hi, int *wo, int *ho);
static void (*convert_point)(float32_t xi, float32_t yi, int *xo, int *yo);
static void (*Display_binding_line)(int x0, int y0, int x1, int y1, uint32_t color);

static void Display_keypoint(spe_pp_outBuffer_t *key, uint32_t color, int* x_value, int* y_value)
{
  int is_clamp;
  int xc, yc;
  int x, y;

  if (key->proba < AI_POSE_PP_CONF_THRESHOLD)
    return ;

  convert_point(key->x_center, key->y_center, &x, &y);
  xc = x - CIRCLE_RADIUS / 2;
  yc = y - CIRCLE_RADIUS / 2;
  is_clamp = clamp_point(&xc, &yc);
  xc = x + CIRCLE_RADIUS / 2;
  yc = y + CIRCLE_RADIUS / 2;
  is_clamp |= clamp_point(&xc, &yc);

  if (is_clamp)
    return ;

  UTIL_LCD_FillCircle(x, y, CIRCLE_RADIUS, color);
  *x_value = x;
  *y_value = y;
}

static void Display_binding(spe_pp_outBuffer_t *from, spe_pp_outBuffer_t *to, uint32_t color)
{
  int is_clamp;
  int x0, y0;
  int x1, y1;
  int i;

  assert(BINDING_WIDTH % 2 == 1);

  if (from->proba < AI_POSE_PP_CONF_THRESHOLD)
    return ;
  if (to->proba < AI_POSE_PP_CONF_THRESHOLD)
    return ;

  convert_point(from->x_center, from->y_center, &x0, &y0);
  is_clamp = clamp_point(&x0, &y0);
  if (is_clamp)
    return ;

  convert_point(to->x_center, to->y_center, &x1, &y1);
  is_clamp = clamp_point(&x1, &y1);
  if (is_clamp)
    return ;

  UTIL_LCD_DrawLine(x0, y0, x1, y1, color);
  for (i = 1; i <= (BINDING_WIDTH - 1) / 2; i++) {
    if (abs(y1 - y0) > abs(x1 - x0)) {
      Display_binding_line(x0 + i, y0, x1 + i , y1, color);
      Display_binding_line(x0 - i, y0, x1 - i , y1, color);
    } else {
      Display_binding_line(x0, y0 + i, x1 , y1 + i, color);
      Display_binding_line(x0, y0 - i, x1 , y1 - i, color);
    }
  }
}

void Display_spe_InitFunctions(int clamp_point_init(int *x, int *y),
                               void convert_length_init(float32_t wi, float32_t hi, int *wo, int *ho),
                               void convert_point_init(float32_t xi, float32_t yi, int *xo, int *yo),
                               void Display_binding_line_init(int x0, int y0, int x1, int y1, uint32_t color))
{
  clamp_point = clamp_point_init;
  convert_length = convert_length_init;
  convert_point = convert_point_init;
  Display_binding_line = Display_binding_line_init;
}
extern int x[13];
extern int y[13];
extern int color[13];
void Display_spe_Detection(spe_pp_outBuffer_t *detect)
{
  int i;

  for (i = 0; i < ARRAY_NB(bindings); i++)
    Display_binding(&detect[bindings[i][0]], &detect[bindings[i][1]], bindings[i][2]);
  for (i = 0; i < AI_POSE_PP_POSE_KEYPOINTS_NB; i++) {
	if (i > 13) {
		Display_keypoint(&detect[i], kp_color[i], NULL, NULL);
	} else {
		Display_keypoint(&detect[i], kp_color[i], &x[i], &y[i]);
		color[i] = kp_color[i];
	}
  }
}
