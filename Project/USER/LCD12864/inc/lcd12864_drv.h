#ifndef __LCD12864_DRV_H__
#define __LCD12864_DRV_H__
#include "stm32f10x_gpio.h"

/*GPIOA*/
#define LCD_RS_PIN GPIO_Pin_11
#define LCD_EN_PIN GPIO_Pin_12
#define LCD_DB0_PIN GPIO_Pin_15
/*GPIOC*/
#define LCD_DB1_PIN GPIO_Pin_10
#define LCD_DB2_PIN GPIO_Pin_11
#define LCD_DB3_PIN GPIO_Pin_12
/*GPIOD*/
#define LCD_DB4_PIN GPIO_Pin_2
/*GPIOB*/
#define LCD_DB5_PIN GPIO_Pin_3
#define LCD_DB6_PIN GPIO_Pin_4
#define LCD_DB7_PIN GPIO_Pin_5
#define LCD_CS1_PIN GPIO_Pin_8
#define LCD_CS2_PIN GPIO_Pin_9

#define __UMEM32(addr) (*(volatile unsigned long*)addr)

#define PASET   __UMEM32(0x40010810)
#define PARESET __UMEM32(0x40010814)

#define PBSET   __UMEM32(0x40010C10)
#define PBRESET __UMEM32(0x40010C14)

#define PCSET   __UMEM32(0x40011010)
#define PCRESET __UMEM32(0x40011014)

#define PDSET   __UMEM32(0x40011410)
#define PDRESET __UMEM32(0x40011414)

#define LCDRS_ON()  PASET = LCD_RS_PIN          //读写数据
#define LCDRS_OFF() PARESET = LCD_RS_PIN        //读写指令

#define LCDEN_ON()  PASET = LCD_EN_PIN          //下降沿所存数据到DDRAM
#define LCDEN_OFF() PARESET = LCD_EN_PIN

#define LCDDB0_ON()  PASET = LCD_DB0_PIN
#define LCDDB0_OFF() PARESET = LCD_DB0_PIN

#define LCDDB1_ON()  PCSET = LCD_DB1_PIN
#define LCDDB1_OFF() PCRESET = LCD_DB1_PIN

#define LCDDB2_ON()  PCSET = LCD_DB2_PIN
#define LCDDB2_OFF() PCRESET = LCD_DB2_PIN

#define LCDDB3_ON()  PCSET = LCD_DB3_PIN
#define LCDDB3_OFF() PCRESET = LCD_DB3_PIN

#define LCDDB4_ON()  PDSET = LCD_DB4_PIN
#define LCDDB4_OFF() PDRESET = LCD_DB4_PIN

#define LCDDB5_ON()  PBSET = LCD_DB5_PIN
#define LCDDB5_OFF() PBRESET = LCD_DB5_PIN

#define LCDDB6_ON()  PBSET = LCD_DB6_PIN
#define LCDDB6_OFF() PBRESET = LCD_DB6_PIN

#define LCDDB7_ON()  PBSET = LCD_DB7_PIN
#define LCDDB7_OFF() PBRESET = LCD_DB7_PIN

#define LCDCS1_ON()  PBSET = LCD_CS1_PIN        //选择芯片右半边屏
#define LCDCS1_OFF() PBRESET = LCD_CS1_PIN

#define LCDCS2_ON()  PBSET = LCD_CS2_PIN        //选择芯片左半边屏
#define LCDCS2_OFF() PBRESET = LCD_CS2_PIN

#define Delay1Us() {for(volatile unsigned char i = 0; i < 10; i++);}




#define LCDDispOn 0x3f          //显示开
#define LCDPageSet 0xb8       //页设置
#define LCDColumnAddrSet 0x40       //列地址设置
#define LCDStartRowAddrSet 0xc0       //起始行设置
#define LCDR 1
#define LCDL 0


typedef struct
{
    unsigned char b0:1;
    unsigned char b1:1;
    unsigned char b2:1;
    unsigned char b3:1;
    unsigned char b4:1;
    unsigned char b5:1;
    unsigned char b6:1;
    unsigned char b7:1;
}BYTE_FIELD;

typedef union
{
    unsigned char byte;
    BYTE_FIELD bit;
}TYPE_BYTE;

typedef struct
{
    unsigned char Buf[9];
    unsigned char Fuhao;
}NumToArrayBufDef;

//#define fgDispMode FLAG0.bit.b0
//#define fgLcdCS1 FLAG3.bit.b0

extern TYPE_BYTE FLAG0;
extern uint8_t fgDispMode;
extern uint8_t menu;
extern unsigned short ClockDir;//顺时针时候为1

extern void LCD12864_HardwareInit(void);
extern void LCD_ShowMenuMain(void);
extern void LcdReset(void);

#endif
