#include "stm32f10x.h"
#include "lcd12864_drv.h"
#include "Key_drv.h"
#include "stm32f10x_it.h"
#include "MC_Hall.h"

void SysTick_Init(void);
static void Task_Run(void);
void main(void)
{
  SysTick_Init();
  LCD12864_HardwareInit();
  Key_HardwareInit();
  HALL_Init(&gHall_Obj);
  LcdReset();
  fgDispMode = 1;
  while(1)
  {
      Task_Run();
  }
}


void SysTick_Init(void)
{
    if (SysTick_Config(SystemCoreClock/2000))
    {
        while(1);
    }
}

static void Task_Run(void)
{
    if (Task_Delay.TimeDelay_500us)
    {
        Task_Delay.TimeDelay_500us = 0;
        if (++Task_Delay.TimeDelay_1ms >= 2)
        {
            Task_Delay.TimeDelay_1ms = 0;
            /*1ms ʱ������*/
            Task_Key();
        }
        if (++Task_Delay.TimeDelay_10ms >= 20)
        {
            Task_Delay.TimeDelay_10ms = 0;
            /*10ms ʱ������*/
            
        }
        if (++Task_Delay.TimeDelay_100ms >= 200)
        {
            Task_Delay.TimeDelay_100ms = 0;
            /*100ms ʱ������*/
            LCD_ShowMenuMain();
        }
        if (++Task_Delay.TimeDelay_500ms >= 1000)
        {
            Task_Delay.TimeDelay_500ms = 0;
            /*500ms ʱ������*/
        }
    }
}

  