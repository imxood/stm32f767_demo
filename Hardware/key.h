#pragma once

#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>

#define KEY0 HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_3)
#define KEY1 HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2)
#define KEY2 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)
#define WK_UP HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

typedef enum {
    NO_KEY_PRESS, KEY0_PRESS, KEY1_PRESS, KEY2_PRESS, WK_UP_PRESS
} KeyStatus;

KeyStatus KEY_scan();
