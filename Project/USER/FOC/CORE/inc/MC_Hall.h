#ifndef __MC_HALL_H__
#define __MC_HALL_H__
#include "MC_Control_Param.h"
#include "mc_hw_hall.h"

#define HALL_SPEED_FIFO_SIZE 6
#define HALL_CKTIM CKTIM

#define HALL_TIMEOUT_TIME  (60)//ms
#define HALL_MIN_NOSIETIME  (250)//us //最大电频率>666HZ时需要调整
#define HALL_DELAY_TIME_US (40)//us

#define NEGATIVE        (-1)
#define UNCHANGE        0
#define POSITIVE        1

extern unsigned char CheckHallValidity(uint8_t hall);
extern void HALL_Init(_Hall_Obj *hall_handler);
extern int16_t Hall_Cal_Base_Angle(_Hall_Obj *hall_handler, uint8_t hall_value);
extern void HALL_CalcSpeed(_Hall_Obj *hall_handler);
#endif
