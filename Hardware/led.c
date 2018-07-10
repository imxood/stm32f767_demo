
#include <led.h>

// 初始化PB0,PB1
void LED_init()
{
    GPIO_InitTypeDef GPIO_init;
    __HAL_RCC_GPIOB_CLK_ENABLE(); // 开启GPIOB时钟

    GPIO_init.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PB0, PB1
    GPIO_init.Mode = GPIO_MODE_OUTPUT_PP;    // 推挽输出
    GPIO_init.Pull = GPIO_PULLUP;            // 上拉
    GPIO_init.Speed = GPIO_SPEED_HIGH;       // 高速

    HAL_GPIO_Init(GPIOB, &GPIO_init);

    LED0(1); // 置0, 灯亮
    LED1(1);
}
