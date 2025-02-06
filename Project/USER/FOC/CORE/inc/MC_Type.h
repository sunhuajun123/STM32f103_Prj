#ifndef __MC_TYPE_H__
#define __MC_TYPE_H__
#include "stm32f10x.h"
#include "stdbool.h"

typedef struct
{
    int32_t hLower_Limit_Output;	 //Lower Limit for Output limitation		���������
    int32_t hUpper_Limit_Output;	 //Lower Limit for Output limitation		���������
    int32_t wLower_Limit_Integral;	 //Lower Limit for Integral term limitation ����������
    int32_t wUpper_Limit_Integral;	 //Lower Limit for Integral term limitation ����������

    int32_t wIntegral;		 //�����ۻ���
    int32_t wPreviousError;  //�ϴ����
    int16_t hKp_Gain;		 		//����ϵ��
    uint16_t hKp_Divisor;		 	//����ϵ������
    int16_t hKi_Gain;		 		//����ϵ��
    uint16_t hKi_Divisor;		 	//����ϵ������
    int16_t hKd_Gain;		 //΢��ϵ��
    uint16_t hKd_Divisor;    //΢��ϵ������
} PID_Struct_t;

typedef enum
{
	HALL_OK = 0,
	NO_HALL_FAULT = 1,
	HALL_WRONG_FAULT = 2,
}_HALL_STATUS;

/*���໷�ýṹ��*/
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
    bool HallTimeOut;/*���HALL��ʱ*/
    volatile bool Noise_Flag;/*��Ǵ˴��ж��Ƿ�Ϊ����*/
    volatile bool HighSpeed_Flag;
    _HALL_STATUS Hall_Status;/*����״̬ 0ok 1�޻��� 2��������*/
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
    int32_t Dst_Cnt;/*HALL�źż���*/
    volatile uint32_t wCaptBuf;/*���񵽵�hall����ֵ*/
    
    int32_t hRotorFreq_dpp;/*ÿ��PWM����ת�ӽǶȱ仯��*/
    int32_t hRotorSpeed;/*ÿ����hall������*/
    int32_t hRotorFreq_Abs_dpp;/*����hallƵ��*/
    int32_t EleAngleCnt;/*����PWM����Ƕ�*/
    int32_t ElSpeedSum;/* Period accumulator used to speed up the average speed computation*/
    uint16_t hCaptCounter;/*�����жϷ����Ĵ���*/
    uint16_t Hall_Shift;/*���BC���Ƿ�Ե�*/
    uint16_t OVFCounter;/*�������*/
    uint16_t hMaxTimerOverflow;/*����������*/
    
    int16_t MeasuredElAngle;
    int16_t hElectrical_Angle;
    int16_t TargetElAngle;
    int16_t Hall_Delta_Angle;/*HallУ׼ֵ�뵱ǰֵ�Ĳ�*/
    int16_t CompSpeed;/*!< Speed compensation factor used to syncronize
			the current electrical angle with the target
			electrical angle. */
    int16_t CurrentSpeed;
    PLL_STRUCT hall_pll;
}_Hall_Obj;

#endif
