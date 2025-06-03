#include "mc_hw_hall.h"
#include "stdint.h"
#include "stm32f10x_gpio.h"
#include "MC_Const.h"
#include "MC_Hall.h"
/****************************************************************************** 
* Function Name    	: Hall_GPIOInit
* Modify/Creat time	: 2025/01/17
* Modified/Creat By	: shj
* Description    	: Hall传感器对应IO端口外设初始化
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void Hall_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);    
    GPIO_InitStructure.GPIO_Pin = MOTOR_HALL_A_PIN | MOTOR_HALL_B_PIN | MOTOR_HALL_C_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_HALL_PORT, &GPIO_InitStructure);
}
/****************************************************************************** 
* Function Name    	: Hall_TIMInit
* Modify/Creat time	: 2025/01/17
* Modified/Creat By	: shj
* Description    	: HALL传感器信号采样对应定时器外设初始化
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void Hall_TIMInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_HALLICInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /*TIM3时钟频率=72M*/
    TIM_DeInit(MOTOR_HALLSENSOR_TIMER);
    TIM_TimeBaseStructure.TIM_Period = UINT16_MAX-1;/*自动装载值*/
    TIM_TimeBaseStructure.TIM_Prescaler = HALL_TIM_MAX_PSC;/*计数分频：计数频率=定时器时钟频率/TIM_Prescaler=72000000/100=720000*/
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*时钟分频：系统时钟作为时钟信号 = 72M*/
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;/*计数模式*/
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;/*重复计数功能关闭*/
    TIM_TimeBaseInit(MOTOR_HALLSENSOR_TIMER, &TIM_TimeBaseStructure);
    
    TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;/*输入捕获通道*/
    TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;/*输入捕获极性：上升沿*/
    TIM_HALLICInitStructure.TIM_ICPrescaler = 0;/*输入信号分频系数：不分频*/
    TIM_HALLICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;/*直接捕获: IC1输入信号来自捕获通道1的TI1FP1*/
    TIM_HALLICInitStructure.TIM_ICFilter = 0x0f;/*输入信号滤波：信号电平变化后连续取0x0f个数据作为信号是否有效的判断依据*/
    TIM_ICInit(MOTOR_HALLSENSOR_TIMER, &TIM_HALLICInitStructure);
    
    TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_2;/*输入捕获通道*/
    TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;/*触发输入捕获极性：下降沿*/
    TIM_HALLICInitStructure.TIM_ICPrescaler = 0;/*输入信号分频系数：不分频*/
    TIM_HALLICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;/*间接捕获：IC2的输入信号来自捕获通道1的TI1FP2*/
    TIM_HALLICInitStructure.TIM_ICFilter = 0x0f;/*输入信号滤波：信号电平变化后连续取0x0f个数据作为信号是否有效的判断依据*/
    TIM_ICInit(MOTOR_HALLSENSOR_TIMER, &TIM_HALLICInitStructure);
    
    TIM_PrescalerConfig(MOTOR_HALLSENSOR_TIMER, 100, TIM_PSCReloadMode_Immediate);/*设置预分频重载模式：立即重载*/
    
    TIM_SelectHallSensor(MOTOR_HALLSENSOR_TIMER, ENABLE);/*使能HALL传感器接口，输入CH1,CH2,CH3经过异或比较器连接到TI1输入*/
    TIM_SelectInputTrigger(MOTOR_HALLSENSOR_TIMER, TIM_TS_TI1FP1);/*输入触发源选择：TI1FP1*/
    
    TIM_UpdateRequestConfig(MOTOR_HALLSENSOR_TIMER, TIM_UpdateSource_Regular);/*定时器更新请求事件：更新事件由定时器向上计数溢出产生*/
    
    TIM_ITConfig(MOTOR_HALLSENSOR_TIMER, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, ENABLE);/*中断使能*/
    
    TIM_Cmd(MOTOR_HALLSENSOR_TIMER, ENABLE);/*定时器使能*/
}

/****************************************************************************** 
* Function Name    	: Hall_TIMNVIC_Init
* Modify/Creat time	: 2025/01/17
* Modified/Creat By	: shj
* Description    	: HALL传感器对应定时器中断外设初始化
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void Hall_TIMNVIC_Init(void)
{
    uint32_t NvicPrio_Group = 0;
    
    NvicPrio_Group = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(HALLTIM_IRQN, NVIC_EncodePriority(NvicPrio_Group, 1, 0));
    NVIC_EnableIRQ(HALLTIM_IRQN);
}

/****************************************************************************** 
* Function Name    	: HallHardware_Init
* Modify/Creat time	: 2025/01/17
* Modified/Creat By	: shj
* Description    	: 初始化HALL硬件相关外设
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HallHardware_Init(void)
{    
    Hall_GPIOInit();
    Hall_TIMInit();
    Hall_TIMNVIC_Init();    
}
/****************************************************************************** 
* Function Name    	: ReadHallState
* Modify/Creat time	: 2025/01/22
* Modified/Creat By	: shj
* Description    	: 获取hall状态值 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
uint8_t hall_a_offset = 0, hall_b_offset = 2, hall_c_offset = 1; //hall 相续调试用
uint8_t ReadHallState(_Hall_Obj *this)
{
    uint16_t hAux1, hAux2, hAux3;
    if (this->Hall_Shift)
    {
        hAux3 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_A_PIN) >> MOTOR_HALL_A_MASK) << hall_a_offset;//2; 
        hAux1 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_B_PIN) >> MOTOR_HALL_B_MASK) << hall_b_offset;//1;
        hAux2 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_C_PIN) >> MOTOR_HALL_C_MASK) << hall_c_offset;//0;
    }
    else
    {
        hAux3 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_A_PIN) >> MOTOR_HALL_A_MASK) << hall_a_offset;//2;
        hAux1 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_C_PIN) >> MOTOR_HALL_C_MASK) << hall_b_offset;//1;
        hAux2 = ((uint16_t)(MOTOR_HALL_PORT->IDR & MOTOR_HALL_B_PIN) >> MOTOR_HALL_B_MASK) << hall_c_offset;//0;
    }
    hAux3 |= hAux1 + hAux2;
    hAux3 &= 0x0007;

    return (uint8_t)hAux3;
}

int16_t Hall_CalBaseAngle(_Hall_Obj *hall_handler,uint8_t hall_value)
{
    int32_t lBaseAngle = 0;
    if (hall_value == hall_handler->bHallCommTab[0])
    {
        lBaseAngle = (int16_t)S16_60_PHASE_SHIFT;
    }
    else if(hall_value == hall_handler->bHallCommTab[1])
    {
        lBaseAngle = (int16_t)S16_120_PHASE_SHIFT;
    }
    else if(hall_value == hall_handler->bHallCommTab[2])
    {
        lBaseAngle = (int16_t)S16_180_PHASE_SHIFT;
    }
    else if(hall_value == hall_handler->bHallCommTab[3])
    {
        lBaseAngle = (int16_t)-S16_120_PHASE_SHIFT;
    }
    else if(hall_value == hall_handler->bHallCommTab[4])
    {
        lBaseAngle = (int16_t)-S16_60_PHASE_SHIFT;
    }
    else if(hall_value == hall_handler->bHallCommTab[5])
    {
        lBaseAngle = 0;
    }
    return lBaseAngle;
}

void Hall_InitElectricalAngle(_Hall_Obj *hall_handler)
{
    hall_handler->Hall_This = ReadHallState(hall_handler);
    if (hall_handler->Hall_This != 0 && hall_handler->Hall_This != 7)
    {
        hall_handler->MeasuredElAngle = Hall_CalBaseAngle(hall_handler,hall_handler->Hall_This);
        hall_handler->TargetElAngle = hall_handler->MeasuredElAngle;
    }
}

void Hall_ClearMeasure(_Hall_Obj *hall_handler)
{
    uint8_t bIndex = 0;
    for (bIndex = 0; bIndex < 6; bIndex++)
    {
        hall_handler->SensorPeriod[bIndex] = 0;
    }
    hall_handler->BufferFilled = 0;
    hall_handler->CurrentSpeed = 0;
    hall_handler->bSpeedFIFO_Index = 1;
    hall_handler->ElSpeedSum = 0;
    hall_handler->hRotorFreq_dpp = 0;
    hall_handler->EleAngleCnt = 0;
    hall_handler->hRotorSpeed = 0;
    hall_handler->OVFCounter = 0;
    hall_handler->CompSpeed = 0;
    hall_handler->HighSpeed_Flag = false;
}

void Hall_timer_div_reset(_Hall_Obj *hall_handler)
{
	uint16_t target_div = 0;//目标分频
		
	target_div = hall_handler->wCaptBuf / 0x7fff;
		
    if (target_div > HALL_TIM_MAX_PSC) //限制分频数
    {
        target_div = HALL_TIM_MAX_PSC;
    }
    else if (target_div < 1)
    {
        target_div = 1;
    }
    target_div -= 1;
    if (target_div != MOTOR_HALLSENSOR_TIMER->PSC)
    {
        MOTOR_HALLSENSOR_TIMER->PSC = target_div;
        MOTOR_HALLSENSOR_TIMER->EGR = TIM_PSCReloadMode_Immediate;//立即更新分频，否则将一个周期才更新
    }

}

/****************************************************************************** 
* Function Name    	: Hall_Timer_Reset
* Modify/Creat time	: 2025/01/21
* Modified/Creat By	: shj
* Description    	: 定时器相关参数初始化 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void Hall_Timer_Reset(TIM_TypeDef * TIMx)
{
    TIMx->SR = 0;
    TIMx->PSC = HALL_TIM_MAX_PSC;
    TIMx->CNT = 0;
}

/****************************************************************************** 
* Function Name    	: Hall_CapValue_Get
* Modify/Creat time	: 2025/01/22
* Modified/Creat By	: shj
* Description    	: 获取HALL扇区切换时定时计数器的计数值 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
uint32_t Hall_CapValue_Get(_Hall_Obj *hall_handler)
{
    uint32_t wCaptBuf	= (uint32_t)(MOTOR_HALLSENSOR_TIMER->CNT);      /*hall扇区切换时对应的定时器计数器值*/
    uint16_t hPrscBuf = MOTOR_HALLSENSOR_TIMER->PSC;                    /*分频因子*/
    /* Add the numbers of overflow to the counter */
    wCaptBuf += (uint32_t)hall_handler->OVFCounter * 0x10000uL;         /*定时器溢出次数 * 定时器装载值*/
    wCaptBuf *= (hPrscBuf + 1u);                                        /*总计数值乘以分频系数*/
    return wCaptBuf;
}


/****************************************************************************** 
* Function Name    	: HallChange_IrqProcess
* Modify/Creat time	: 2025/01/22
* Modified/Creat By	: shj
* Description    	: hall扇区切换处理 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void HallChange_IrqProcess(_Hall_Obj* hall_handler)
{
    hall_handler->Noise_Flag = false;
    
    if (true == CheckHallValidity(hall_handler->Hall_This))
    {
        if (hall_handler->Hall_Prev == UINT8_MAX)
        {
            /*第一次捕获*/
            hall_handler->bSpeed = UNCHANGE;
            hall_handler->hCaptCounter = 0; 
        }
        else
        {
            int8_t direct_this = UNCHANGE;
            if (hall_handler->Hall_This == hall_handler->bHallPossableTab[hall_handler->Hall_Prev])
            {
                direct_this = POSITIVE;
            }
            else if (hall_handler->Hall_This == hall_handler->bHallPossableTab2[hall_handler->Hall_Prev])
            {
                direct_this = NEGATIVE;
            }
            else
            {
                direct_this = UNCHANGE;
            }
            if (hall_handler->bSpeed != direct_this)
            {
                if (hall_handler->hRotorFreq_Abs_dpp > 300)
                {
                    hall_handler->Noise_Flag = true;
                }
                else
                {                
                    hall_handler->bSpeed = direct_this;            
                }
            }
        }
        hall_handler->Hall_Prev = hall_handler->Hall_This;
    }
    else
    {
        hall_handler->Noise_Flag = true;
    }
    
    int32_t E_agl_t;
    if (false == hall_handler->Noise_Flag)
    {
        int32_t time_delay_comp = hall_handler->hRotorFreq_dpp*SAMPLING_FREQ*HALL_DELAY_TIME_US/1000000;
        E_agl_t =  Hall_Cal_Base_Angle(hall_handler,hall_handler->Hall_This);
		switch (hall_handler->bSpeed)
		{
			case POSITIVE:
				E_agl_t = E_agl_t;
				break;
			case NEGATIVE:
				E_agl_t += S16_60_PHASE_SHIFT;
				break;
			default:
				E_agl_t += (S16_60_PHASE_SHIFT / 2);
				break;
		}
        E_agl_t += time_delay_comp;
       
        if (hall_handler->HighSpeed_Flag == false) 
		{
			hall_handler->MeasuredElAngle = E_agl_t;
	
		}
		else//高速时霍尔异常，不进行校正
		{
			if (hall_handler->Hall_This == 5) //360度校准一次
			{
				hall_handler->MeasuredElAngle = E_agl_t;
			}
		}
        
        if (hall_handler->HallTimeOut)
		{
			hall_handler->HallTimeOut = false;
			Hall_ClearMeasure(hall_handler);
			hall_handler->TargetElAngle = hall_handler->MeasuredElAngle;//强制校正
			hall_handler->HallStart_Flag = 1;
			hall_handler->HallStart_Step = 100;
			hall_handler->AfterTimeOut_Cnt = 0;
		}
		else
		{
			if (hall_handler->AfterTimeOut_Cnt < 1)//发生超时之后，第二个脉冲
			{
				hall_handler->AfterTimeOut_Cnt++;
				hall_handler->TargetElAngle = hall_handler->MeasuredElAngle;//强制校正
			}
	

			if (hall_handler->wCaptBuf < HALL_MIN_NOSIETIME * (HALL_CKTIM / 1000000))
			{
	
			}
			else
			{
				Hall_timer_div_reset(hall_handler);
				//存储捕获脉宽
				if (++hall_handler->bSpeedFIFO_Index > (HALL_SPEED_FIFO_SIZE - 1))
				{
					hall_handler->bSpeedFIFO_Index = 0;
				}
				hall_handler->Hall_Puse[hall_handler->bSpeedFIFO_Index] = hall_handler->wCaptBuf;
	
	
				if (hall_handler->HighSpeed_Flag == false)
				{
					/* Update pointers to speed buffer */
					hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index] = (int32_t)(hall_handler->PseudoFreqConv / hall_handler->wCaptBuf);
					hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index] *= hall_handler->bSpeed;//bSpeed=0/1/-1(转向)
					hall_handler->CurrentSpeed = hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index];
	
					HALL_CalcSpeed(hall_handler);
				}
				else
				{
					if (hall_handler->Hall_This == 5) //360度校准一次。用于处理电频率高的情况。减少处理时间
					{
						int32_t sum = 0;
						for (int i = 0; i < HALL_SPEED_FIFO_SIZE; i++)
						{
							sum += hall_handler->Hall_Puse[i];
						}
						sum /= HALL_SPEED_FIFO_SIZE;
	
						hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index] = (int32_t)(hall_handler->PseudoFreqConv / sum);
						hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index] *= hall_handler->bSpeed;//bSpeed=0/1/-1(转向)
						hall_handler->CurrentSpeed = hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index];
						HALL_CalcSpeed(hall_handler);
					}
				}
			}
		
		}
        if (hall_handler->hCaptCounter < UINT16_MAX)
		{
			hall_handler->hCaptCounter++;
		}
    }
    
    hall_handler->OVFCounter = 0;
}

/****************************************************************************** 
* Function Name    	: HallOverTime_IrqProcess
* Modify/Creat time	: 2025/01/22
* Modified/Creat By	: shj
* Description    	: hall扇区切换超时处理 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void HallOverTime_IrqProcess(_Hall_Obj* hall_handler)
{
    hall_handler->OVFCounter++;
    if (hall_handler->OVFCounter > hall_handler->hMaxTimerOverflow)
    {
        hall_handler->HallTimeOut = true;
        hall_handler->OVFCounter = 0;
        Hall_InitElectricalAngle(hall_handler);
        Hall_ClearMeasure(hall_handler);
    }
}

/****************************************************************************** 
* Function Name    	: TIM3_IRQHandler
* Modify/Creat time	: 2025/01/21
* Modified/Creat By	: shj
* Description    	: HALL捕获定时器中断
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void TIM3_IRQHandler(void)
{
    _Hall_Obj* lthis = &gHall_Obj;
    if (TIM_GetFlagStatus(MOTOR_HALLSENSOR_TIMER,TIM_FLAG_CC1 | TIM_FLAG_CC2) == SET)
    {
        /*
        ** 发生输入捕获中断
        */
        MOTOR_HALLSENSOR_TIMER->SR = 0;                 /*清除中断标志*/
        lthis->wCaptBuf = Hall_CapValue_Get(lthis);      /*获取定时器计数值*/
        MOTOR_HALLSENSOR_TIMER->CNT = 0;                /*清除定时计数器值*/
        lthis->Hall_This = ReadHallState(lthis);         /*获取hall状态*/
        HallChange_IrqProcess(lthis);        
    }
    else
    {
        /*
        ** 发生溢出中断
        */
        lthis->hMaxTimerOverflow = (HALL_TIMEOUT_TIME * (HALL_CKTIM / 65536)) / ((MOTOR_HALLSENSOR_TIMER->PSC + 1) * 1000);
        HallOverTime_IrqProcess(lthis);
    }
    MOTOR_HALLSENSOR_TIMER->SR = 0;/*清除中断标志*/
}
