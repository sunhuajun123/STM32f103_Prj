#include "Key_drv.h"
#include "lcd12864_drv.h"
#include "MC_Type.h"

Key_Struct_ gKey_Struct;

void Key_HardwareInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |\
      RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = KEY_LEFT_PIN | KEY_FUNC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = KEY_RIGHT_PIN | KEY_DOWN_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = KEY_UP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Proc_LeftKey(void)
{
    if(menu==0)
    {
	ClockDir=1;	
        fgDispMode=1;
//	hSpeed_Reference += (u16)(1500/6);
    }
    if(menu==2)
    {
//        hSpeed_Reference += 10;
        fgDispMode=1;
    }
    if(menu==4)
    {
        pFOC_ctr_handle.hTorque_Reference +=100;
        fgDispMode=1;
    }
}

void Proc_RightKey(void)
{
    if(menu==0)
    {
        ClockDir=0; 
        fgDispMode=1;
//	hSpeed_Reference -=(u16)(1500/6);
    }
    if(menu==2)
    {
//	hSpeed_Reference -= 10;		
        fgDispMode=1;
    }
    if(menu==4)
    {
        pFOC_ctr_handle.hTorque_Reference -=100;
        fgDispMode=1;
    }
}

void Proc_UpKey(void)
{    
    if(menu < 7)
    {
        menu++;
    }
    else
    {
        menu=0;
    }
    fgDispMode=1;
}

void Proc_DownKey(void)
{
    if(menu > 0)
    {
        menu--;
    }
    else
    {
        menu=7;
    }
    fgDispMode=1;
}

void Proc_FuncKey(void)
{
    
}
/****************************************************************************** 
* Function Name    	: KeyVal_Get
* Modify/Creat time	: 2025/04/13
* Modified/Creat By	: shj
* Description    	: 按键状态获取
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void KeyVal_Get(Key_Struct_* Key_Handle)
{
    #define KEY_FILTER_TIME 40
    static uint8_t sKeyFilter_Cnt = 0;
    KEY_STA_ sKeySta_Get = KEY_IDLE;
    
    if (!LEFTKEY_STA_GET())
    {
        sKeySta_Get = KEY_LEFT;
    }
    else if (!RIGHTKEY_STA_GET())
    {
        sKeySta_Get = KEY_RIGHT;
    }
//    else if (!UPKEY_STA_GET())
//    {
//        sKeySta_Get = KEY_UP;
//    }
    else if (!DOWNKEY_STA_GET())
    {
        sKeySta_Get = KEY_DOWN;
    }
    else if (!FUNCKEY_STA_GET())
    {
        sKeySta_Get = KEY_FUNC;
    }
    else
    {
        if (sKeyFilter_Cnt >= KEY_FILTER_TIME)
        {
            sKeyFilter_Cnt = 0;
            Key_Handle->Key_Sta = Key_Handle->KeySta_Filter;            
        }
        else
        {
            sKeyFilter_Cnt = 0;
            Key_Handle->Key_Sta = KEY_IDLE;
        }
    }
    /*按键状态滤波*/
    if (Key_Handle->KeySta_Filter != sKeySta_Get)
    {        
        Key_Handle->KeySta_Filter = sKeySta_Get;
        sKeyFilter_Cnt = 0;
    } 
    else
    {
        if (sKeyFilter_Cnt < KEY_FILTER_TIME)
        {
            sKeyFilter_Cnt++;
        }
    }    
}


/****************************************************************************** 
* Function Name    	: Task_Key
* Modify/Creat time	: 2025/04/13
* Modified/Creat By	: shj
* Description    	: 按键任务
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void Task_Key(void)
{  
    KeyVal_Get(&gKey_Struct);/*检测实时按键状态*/
    switch(gKey_Struct.Key_Sta)
    {
    case KEY_IDLE:
      
      break;
    case KEY_LEFT:
      Proc_LeftKey();   /*改变运行方向，增加运行速度*/
      break;
    case KEY_RIGHT:
      Proc_RightKey();  /*改变运行方向，减小运行速度*/
      break;
    case KEY_UP:
      Proc_UpKey();     /*显示菜单页面切换*/
      break;
    case KEY_DOWN:
      Proc_DownKey();   /*显示菜单页面切换*/
      break;
    case KEY_FUNC:
      Proc_FuncKey();   /*控制电机启动和停止*/
      break;
    default:break;
    }
}
