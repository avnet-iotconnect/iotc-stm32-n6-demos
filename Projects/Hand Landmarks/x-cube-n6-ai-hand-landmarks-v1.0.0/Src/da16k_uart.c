/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : custom_bus.c
  * @brief          : source file for the BSP BUS IO driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "da16k_uart.h"
#include "da16k_comm.h"

extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *da_uart_p = &huart2;

#define BUF_SIZE (256)
static uint32_t timout;
#define SET_TIMEOUT() (timout= HAL_GetTick() + DA16K_UART_TIMEOUT_MS) //set 500ms timeout
#define GET_TIMEOUT() (timout < HAL_GetTick()) //check if timeout has expired

bool uart_busy = false;
static volatile char rx_buf[BUF_SIZE];
static int head = 0;
static int tail = 0;

static char rx_byte;

/*
 * Callback that occurs at the end of a transmission.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == da_uart_p) {
        uart_busy = false;
    }
}
/*
 * Callback that happens when characters are received via interrupt one at a time.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == da_uart_p) {
        HAL_UART_Receive_IT(da_uart_p, (uint8_t *) &rx_byte, 1);
        rx_buf[head] = rx_byte;
        if(++head >= BUF_SIZE) {
            head = 0;
        }
    }
}
/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
}

/******************************************************************************/
/* STM32N6xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32n6xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


bool da16k_uart_init(void) {
	HAL_UART_Receive_IT(da_uart_p, (uint8_t *) &rx_byte, 1);
    return true;
}

void da16k_uart_close(void) {
    return;
}

bool da16k_uart_send(const char *src, size_t length) {
    static char tx_buf[2][BUF_SIZE];
    static int buf_select = 0;
    HAL_StatusTypeDef uart_status;

    if(!src || length==0) {
        return false;
    }

    SET_TIMEOUT();
    while(uart_busy){
        if(GET_TIMEOUT()) {
            return false;
        }
    }
    uart_busy = true;

    memcpy(tx_buf[buf_select], src, length);
    do {
    	uart_status = HAL_UART_Transmit_IT(da_uart_p, (uint8_t*)tx_buf[buf_select], length);
    }while(uart_status != HAL_OK);

    buf_select = buf_select ? 0:1;

    return true;
}

/*
 * Function required by the AT cmd lib. This function reads character from the receive buffer of the serial port. If
 * there are no characters to be read the code will block here for the specified timeout length.
 */
da16k_err_t da16k_uart_get_char(char *dst, uint32_t timeout_ms) {
    if(!dst) {
        return DA16K_INVALID_PARAMETER;
    }

    uint32_t expiry = HAL_GetTick() + timeout_ms;

    do {
        if(tail != head) {
            *dst = rx_buf[tail];
            if(++tail >= BUF_SIZE) {
                tail = 0;
            }
            return DA16K_SUCCESS;
        }
    }while(HAL_GetTick() < expiry);

    return DA16K_TIMEOUT;
}
