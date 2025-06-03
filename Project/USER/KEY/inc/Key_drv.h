#ifndef __KEY_DRV_H__
#define __KEY_DRV_H__
#include "stm32f10x_gpio.h"

#define KEY_LEFT_PIN    GPIO_Pin_5      //PC5
#define KEY_RIGHT_PIN   GPIO_Pin_11     //PB11
#define KEY_UP_PIN      GPIO_Pin_14     //PA14
#define KEY_DOWN_PIN    GPIO_Pin_10     //PB10
#define KEY_FUNC_PIN    GPIO_Pin_15     //PC15

#define LEFTKEY_STA_GET() GPIO_ReadInputDataBit(GPIOC, KEY_LEFT_PIN)
#define RIGHTKEY_STA_GET() GPIO_ReadInputDataBit(GPIOB, KEY_RIGHT_PIN)
#define UPKEY_STA_GET() GPIO_ReadInputDataBit(GPIOA, KEY_UP_PIN)
#define DOWNKEY_STA_GET() GPIO_ReadInputDataBit(GPIOB, KEY_DOWN_PIN)
#define FUNCKEY_STA_GET() GPIO_ReadInputDataBit(GPIOC, KEY_FUNC_PIN)

typedef enum
{
    KEY_IDLE = 0,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_UP,
    KEY_DOWN,
    KEY_FUNC,
}KEY_STA_;

typedef struct
{
    KEY_STA_ Key_Sta;
    KEY_STA_ KeySta_Filter;
    KEY_STA_ KeySta_Last;
    uint8_t KeyStaChk_Cnt;
    
}Key_Struct_;

extern void Key_HardwareInit(void);
extern void Task_Key(void);

#endif
