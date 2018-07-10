#pragma once

#include <stdio.h>
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_uart.h>

// 将printf重定向到串口
int fputc(int ch, FILE *f);

// 接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

void uart_main(void);

void uart_init();
