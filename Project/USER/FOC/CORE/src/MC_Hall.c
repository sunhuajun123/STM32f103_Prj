#include "MC_Hall.h"
#include "MC_Type.h"


/****************************************************************************** 
* Function Name    	: hall_comm_table_init
* Modify/Creat time	: 2025/02/24
* Modified/Creat By	: shj
* Description    	: 根据换向表提前计算出上一个霍尔以及下一个霍尔值
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void hall_comm_table_init(_Hall_Obj* hall_handler)
{
    u8 t_i, t_j;

    for (t_i = 1; t_i < 7; t_i++)
    {
        for (t_j = 0; t_j < 6; t_j++)
        {
            if (hall_handler->bHallCommTab[t_j] == t_i)
            {
                if (t_j != 5)
                {
                    hall_handler->bHallPossableTab[t_i] = hall_handler->bHallCommTab[t_j + 1];
                }
                else
                {
                    hall_handler->bHallPossableTab[t_i] = hall_handler->bHallCommTab[0];
                }
				
                break;
            }
        }

        for (t_j = 0; t_j < 6; t_j++)
        {
            if (hall_handler->bHallCommTab[t_j] == t_i)
            {
                if (t_j == 0)
                {
                    hall_handler->bHallPossableTab2[t_i] = hall_handler->bHallCommTab[5];
                }
                else
                {
                    hall_handler->bHallPossableTab2[t_i] = hall_handler->bHallCommTab[t_j - 1];
                }
				
                break;
            }
        }
    }
} 

/****************************************************************************** 
* Function Name    	: HALL_InitHallMeasure
* Modify/Creat time	: 2025/02/24
* Modified/Creat By	: shj
* Description    	: Clear software FIFO where are "pushed" latest speed information
*                       This function must be called before starting the motor to initialize
*	                the speed measurement process. 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HALL_InitHallMeasure(_Hall_Obj* hall_handler)
{
    hall_handler->PseudoFreqConv = ((uint64_t)(HALL_CKTIM / 6) * 0x10000 / (SAMPLING_FREQ));
    hall_handler->OVFCounter = 0u;
    hall_handler->HallTimeOut = false;
    hall_handler->hRotorSpeed = 0;
    hall_handler->hCaptCounter = 0;
    hall_handler->hRotorFreq_dpp = 0;
    hall_handler->hRotorFreq_Abs_dpp = 0;
    hall_handler->BufferFilled = 0;
    hall_handler->HallStart_Flag = 0;
    hall_handler->HallStart_Step = 0;
    hall_handler->AfterTimeOut_Cnt = 0;
    for (hall_handler->bSpeedFIFO_Index = 0; hall_handler->bSpeedFIFO_Index < HALL_SPEED_FIFO_SIZE; hall_handler->bSpeedFIFO_Index++)
    {
        hall_handler->SensorPeriod[hall_handler->bSpeedFIFO_Index] = 0;
    }
    // First measurement will be stored in the 1st array location
    hall_handler->bSpeedFIFO_Index = HALL_SPEED_FIFO_SIZE - 1;
    hall_handler->wCaptBuf = (HALL_TIMEOUT_TIME * (HALL_CKTIM / 1000));
    hall_comm_table_init(hall_handler);
    Hall_Timer_Reset(MOTOR_HALLSENSOR_TIMER);
}

/****************************************************************************** 
* Function Name    	: Hall_Cal_Base_Angle
* Modify/Creat time	: 2024/05/14
* Modified/Creat By	: shj
* Description    	: 计算基础电角度 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
int16_t Hall_Cal_Base_Angle(_Hall_Obj *hall_handler, uint8_t hall_value)
{
    int32_t base_angle = 0;
    if(hall_value==hall_handler->bHallCommTab[0])
    {
        base_angle = (int16_t)S16_60_PHASE_SHIFT;	
    }
    else if(hall_value==hall_handler->bHallCommTab[1])
    {
        base_angle = (int16_t)S16_120_PHASE_SHIFT;	
    }
    else if(hall_value==hall_handler->bHallCommTab[2])
    {
        base_angle = (int16_t)S16_180_PHASE_SHIFT;	
    }
    else if(hall_value==hall_handler->bHallCommTab[3])
    {
        base_angle = (int16_t)-S16_120_PHASE_SHIFT;	
    }
    else if(hall_value==hall_handler->bHallCommTab[4])
    {
        base_angle = (int16_t)-S16_60_PHASE_SHIFT;	
    }
    else if(hall_value==hall_handler->bHallCommTab[5])
    {
        base_angle = 0;	
    }
	
    base_angle += hall_handler->Hall_Offset;

    return base_angle;
}

/****************************************************************************** 
* Function Name    	: CheckHallValidity
* Modify/Creat time	: 2025/02/24
* Modified/Creat By	: shj
* Description    	: 检测hall信号是否有效
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
unsigned char CheckHallValidity(uint8_t hall)
{
    return ((hall != 0) && (hall != 7));
}

/****************************************************************************** 
* Function Name    	: HALL_Init_Electrical_Angle
* Modify/Creat time	: 2025/02/24
* Modified/Creat By	: shj
* Description    	: Read the logic level of the three Hall sensor and individuates
*                         this way the position of the rotor (+/- 30?. Electrical angle
*                         variable is then initialized 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HALL_Init_Electrical_Angle(_Hall_Obj *hall_handler)
{
    hall_handler->Hall_This = ReadHallState(hall_handler);   //霍尔状态读取
    if (true == CheckHallValidity(hall_handler->Hall_This))
    {
        hall_handler->MeasuredElAngle =  Hall_Cal_Base_Angle(hall_handler, \
          hall_handler->Hall_This)+(S16_60_PHASE_SHIFT / 2);
        hall_handler->TargetElAngle = hall_handler->MeasuredElAngle;//强制校正
    }
}

/*******************************************************************************
* Function Name  : int_abs
* Created        : 2014/7/20, by xuezubo
* Description    : None
* Input          : int32_t dat
* Output         : 绝对值
* Return         : int32_t
*******************************************************************************/
int32_t int_abs(int32_t dat)
{
    if (dat < 0)
        return -dat;
    return dat;
}

//计算角度误差，电角度-180=-32768 ，180 = 32767
int16_t angle_diff(int16_t angle1, int16_t angle2)
{
	int32_t theata_err = (int32_t)angle1 - (int32_t)angle2;

	if (theata_err > 0x7fff)
    {
        theata_err -= 0xffff;
    }
    else if (theata_err < -0x7fff)
    {
        theata_err += 0x10000;
    }

    return (int16_t)theata_err;
}

void HALL_CalcSpeed(_Hall_Obj *hall_handler)
{
    if (hall_handler->hCaptCounter == 0)// First capture must be discarded
    {
        hall_handler->hRotorSpeed = 0;
        return;
    }
    else
    {


        if (hall_handler->BufferFilled < HALL_SPEED_FIFO_SIZE)
        {
            hall_handler->BufferFilled++;
            hall_handler->ElSpeedSum += hall_handler->CurrentSpeed;
            hall_handler->hRotorFreq_dpp = hall_handler->CurrentSpeed; /* Average value =PseudoFreqConv/wCaptBuf*/

        }
        else
        {
            hall_handler->ElSpeedSum -= hall_handler->ElSpeedSum / HALL_SPEED_FIFO_SIZE; //
            hall_handler->ElSpeedSum += hall_handler->CurrentSpeed;
            hall_handler->hRotorFreq_dpp = (hall_handler->ElSpeedSum / HALL_SPEED_FIFO_SIZE); /* Average value */

        }
        hall_handler->hRotorFreq_Abs_dpp = int_abs(hall_handler->hRotorFreq_dpp);
        //speed = 角度累加值/时间(即1/PWM_FREQ);单位：转/min
        //this->hRotorSpeed = (int32_t)(((int64_t)this->hall_pll.pll_dpp * 60 * PWM_FREQ) / 0x10000);
        hall_handler->hRotorSpeed = (int32_t)(((int64_t)hall_handler->ElSpeedSum * 10 * PWM_FREQ) / 0x10000);
        
        hall_handler->Hall_Delta_Angle = angle_diff(hall_handler->MeasuredElAngle, hall_handler->TargetElAngle);



        if (hall_handler->HighSpeed_Flag)
        {
            if ((hall_handler->hRotorFreq_Abs_dpp < 500) && (hall_handler->Hall_This == 5))
            {
                hall_handler->HighSpeed_Flag = false;
            }
        }
        else
        {
            if ((hall_handler->hRotorFreq_Abs_dpp > 1200) && (hall_handler->Hall_This == 5))
            {
                hall_handler->HighSpeed_Flag = true;
            }

        }
        //    if(this->high_speed_flag)
        //    {
        //    this->EleAngleCnt = S16_360_PHASE_SHIFT/this->hRotorFreq_Abs_dpp;
        //
        //    }
        //   else
        {
            hall_handler->EleAngleCnt = S16_60_PHASE_SHIFT / hall_handler->hRotorFreq_Abs_dpp;

        }

        if (hall_handler->EleAngleCnt > 0)
        {
            hall_handler->CompSpeed = hall_handler->Hall_Delta_Angle / hall_handler->EleAngleCnt; //*200/(SAMPLING_FREQ);
        }
    }
}

/****************************************************************************** 
* Function Name    	: HALL_Init
* Modify/Creat time	: 2025/02/24
* Modified/Creat By	: shj
* Description    	: hall传感器相关硬件及参数初始化
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HALL_Init(_Hall_Obj *hall_handler)
{
    HallHardware_Init();
    HALL_InitHallMeasure(hall_handler);
    HALL_Init_Electrical_Angle(hall_handler);
}




