/*
 UART AT COMMANDS TEMPLATE
 */
#include "da16k_uart.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

bool da16k_uart_init(void) {
	return true;
}

void da16k_uart_close(void) {
}

bool da16k_uart_send(const char *src, size_t length) {
	HAL_StatusTypeDef USART_STATUS = HAL_OK;
	USART_STATUS = HAL_UART_Transmit(&huart2, (uint8_t* )src, length, DA16K_UART_TIMEOUT_MS);
	if (USART_STATUS != HAL_OK) {
		return false;
	}
	return true;
}

da16k_err_t da16k_uart_get_char(char *dst, uint32_t timeout_ms) {
	HAL_StatusTypeDef USART_STATUS = HAL_OK;
	USART_STATUS = HAL_UART_Receive(&huart2, (uint8_t* )dst, 1, timeout_ms);

	if (USART_STATUS == HAL_OK) {
		return DA16K_SUCCESS;
	}
	else
		return DA16K_TIMEOUT;
}

