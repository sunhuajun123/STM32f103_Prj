#ifndef __MC_TYPE_H__
#define __MC_TYPE_H__
#include "stm32f10x.h"
#include "stdbool.h"

typedef struct
{
    int32_t hLower_Limit_Output;	 //Lower Limit for Output limitation		总输出下限
    int32_t hUpper_Limit_Output;	 //Lower Limit for Output limitation		总输出上限
    int32_t wLower_Limit_Integral;	 //Lower Limit for Integral term limitation 积分项下限
    int32_t wUpper_Limit_Integral;	 //Lower Limit for Integral term limitation 积分项上限

    int32_t wIntegral;		 //积分累积和
    int32_t wPreviousError;  //上次误差
    int16_t hKp_Gain;		 		//比例系数
    uint16_t hKp_Divisor;		 	//比例系数因子
    int16_t hKi_Gain;		 		//积分系数
    uint16_t hKi_Divisor;		 	//积分系数因子
    int16_t hKd_Gain;		 //微分系数
    uint16_t hKd_Divisor;    //微分系数因子
} PID_Struct_t;

typedef enum
{
	HALL_OK = 0,
	NO_HALL_FAULT = 1,
	HALL_WRONG_FAULT = 2,
}_HALL_STATUS;

/*锁相环用结构体*/
typedef struct  
{
    PID_Struct_t pll_pi;
    int16_t pll_dpp;
    int16_t pll_angle;
} PLL_STRUCT;

typedef struct
{
    int8_t bSpeed;
    volatile uint8_t BufferFilled;
    bool HallTimeOut;/*标记HALL超时*/
    volatile bool Noise_Flag;/*标记此次中断是否为噪声*/
    volatile bool HighSpeed_Flag;
    _HALL_STATUS Hall_Status;/*霍尔状态 0ok 1无霍尔 2霍尔错误*/
    uint8_t Hall_This;
    uint8_t Hall_Prev;
    uint8_t HallStart_Falg;
    uint8_t AfterTimeOut_Cnt;
    uint8_t bSpeedFIFO_Index;
    uint32_t PseudoFreqConv;/*!< Conversion factor between time interval Delta T
                            between HALL sensors captures, express in timer
                            counts, and electrical rotor speed express in dpp.
                            Ex. Rotor speed (dpp) = wPseudoFreqConv / Delta T
                            It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/
    int32_t SensorPeriod[6];
    int32_t Hall_Puse[6];
    uint8_t bHallPossableTab[8];
    uint8_t bHallPossableTab2[8];
    uint8_t bHallCommTab[8];
    int32_t s32HallPhaseShift[7];
    int32_t Hall_Offset;
    int32_t HallStart_Step;
    int32_t Dst_Cnt;/*HALL信号计数*/
    volatile uint32_t wCaptBuf;/*捕获到的hall脉宽值*/
    
    int32_t hRotorFreq_dpp;/*每个PWM周期转子角度变化量*/
    int32_t hRotorSpeed;/*每分钟hall周期数*/
    int32_t hRotorFreq_Abs_dpp;/*定义hall频率*/
    int32_t EleAngleCnt;/*用于PWM计算角度*/
    int32_t ElSpeedSum;/* Period accumulator used to speed up the average speed computation*/
    uint16_t hCaptCounter;/*捕获中断发生的次数*/
    uint16_t Hall_Shift;/*标记BC相是否对调*/
    uint16_t OVFCounter;/*溢出次数*/
    uint16_t hMaxTimerOverflow;/*最大溢出次数*/
    
    int16_t MeasuredElAngle;
    int16_t hElectrical_Angle;
    int16_t TargetElAngle;
    int16_t Hall_Delta_Angle;/*Hall校准值与当前值的差*/
    int16_t CompSpeed;/*!< Speed compensation factor used to syncronize
			the current electrical angle with the target
			electrical angle. */
    int16_t CurrentSpeed;
    PLL_STRUCT hall_pll;
}_Hall_Obj;

#endif
