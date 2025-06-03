#include "MC_Type.h" 
#include "mc_hw_hall.h"

_Hall_Obj gHall_Obj =
{
	.bHallCommTab = HALL_SEQUE,
	.Hall_Offset = (int32_t)(MAIN_MOTOR_HALL_PHASE_OFFSET * 0x10000 / 360),
	.Hall_Shift = 0,
	.Hall_Prev = UINT8_MAX,
	.Hall_This = UINT8_MAX,
	.Noise_Flag = 0,
	.hRotorSpeed = 0,
	.hCaptCounter = 0,
	.hRotorFreq_dpp = 0,
	.hElectrical_Angle = 0,
	.hRotorFreq_Abs_dpp = 0,
	.bSpeedFIFO_Index = 0,
	.bSpeed = 0,
};

_FOC_CtrProc pFOC_ctr_handle;