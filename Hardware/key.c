#include <key.h>

KeyStatus KEY_scan()
{
    static uint8_t key_up = 1; // 松开标志

    // 处于松开状态,则不判断按下
    if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || WK_UP == 1))
    {
        HAL_Delay(10); //延时10ms,后依然处于按下状态

        key_up = 0;

        if (KEY0 == 0)
            return KEY0_PRESS;
        if (KEY1 == 0)
            return KEY1_PRESS;
        if (KEY2 == 0)
            return KEY2_PRESS;
        if (WK_UP == 1)
            return WK_UP_PRESS;
    }
    // 判断松开
    else if(KEY0 == 1 && KEY1 == 1 && KEY2 == 1 && WK_UP == 0){
        key_up = 1;
    }
    return NO_KEY_PRESS;
}
