#ifndef __MC_HW_HALL_H__
#define __MC_HW_HALL_H__
#include "stm32f10x_tim.h"
#include "MC_Type.h"

#define HALL_TIMEOUT_TIME (60)//ms

#define MOTOR_HALL_PORT GPIOC

#define MOTOR_HALL_A_MASK GPIO_PinSource6
#define MOTOR_HALL_A_PIN GPIO_Pin_6
#define MOTOR_HALL_B_MASK GPIO_PinSource7
#define MOTOR_HALL_B_PIN GPIO_Pin_7
#define MOTOR_HALL_C_MASK GPIO_PinSource8
#define MOTOR_HALL_C_PIN GPIO_Pin_8

#define MOTOR_HALLSENSOR_TIMER TIM3
#define HALLTIM_IRQN TIM3_IRQn

#define HALL_TIM_MAX_PSC 100

#define HALL_SEQUE {5,1,3,2,6,4,0,0}
#define MAIN_MOTOR_HALL_PHASE_OFFSET 0


#define S16_360_PHASE_SHIFT 65535
#define S16_300_PHASE_SHIFT 54613
#define S16_240_PHASE_SHIFT 43691
#define S16_180_PHASE_SHIFT (65535/2)
#define S16_120_PHASE_SHIFT (65535/3)
#define S16_60_PHASE_SHIFT (16384)

extern void HallHardware_Init(void);
extern uint8_t ReadHallState(_Hall_Obj *this);
extern void Hall_Timer_Reset(TIM_TypeDef * TIMx);
#endif
