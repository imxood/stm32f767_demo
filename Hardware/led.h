#pragma once

#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>

typedef enum {
    LED_ON, LED_OFF
} LedStatus;

#define LED0(n) (n ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET))
#define LED1(n) (n ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET))

void LED_init();
