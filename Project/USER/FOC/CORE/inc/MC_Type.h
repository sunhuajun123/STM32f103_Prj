#ifndef __MC_TYPE_H__
#define __MC_TYPE_H__
#include "stm32f10x.h"
#include "stdbool.h"

typedef struct
{
    int16_t a;
    int16_t b;
} ab_t;

typedef struct
{
    int32_t q;
    int32_t d;
} dq_t;

typedef struct
{
  int32_t alpha;
  int32_t beta;
} alphabeta_t;

typedef struct
{
    int16_t hCos;
    int16_t hSin;
} Trig_Components;

typedef struct
{
    s32 yk_1;
    u16 coef;
} RC_f;

typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} su16UVW;

typedef struct
{
    uint32_t  TimePhA;
    uint32_t  TimePhB;
    uint32_t  TimePhC;
} PWM_TIME;

typedef enum
{
	M1 = 0,//主电机
	M2 = 1,//从电机
}_MOTOR_NUM;//标记哪个电机

typedef enum
{
    INIT = 0,       //初始化
    IDLE = 1,
    STOP = 2,
    WAIT = 3,
    FAULT = 4,
    ALIGNED = 5,    //对齐
    OPENLOOP = 6,   //开环启动
    START =7,
    IPD = 8,
    RUN,
    TORQUE_RUN,
    BRAKE,
    LOCK,
    COASTING,
} SystStatus_t;

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
    uint8_t HallStart_Flag;
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

typedef struct 
{
    _MOTOR_NUM motor_num;
//	_MOTOR_RUN_MODE run_mode;

//	const u16ADC_OFFSET ADC_inidat_dft;
    ab_t Stat_Curr_ab_lowpass;
    dq_t Stat_Curr_qd_r_r;   //电流环的给定值，用于电流环Id,Iq和前馈电流控制的给定值
    ab_t Stat_Curr_ab;
    alphabeta_t Stat_Curr_alfa_beta;
    dq_t Stat_Curr_qd;
    dq_t Stat_Curr_qd_lowpass;
    dq_t Stat_Volt_qd;
    alphabeta_t Stat_Volt_alfa_beta;
    Trig_Components Vector;
    uint16_t cntFirCurA;//滤波舍弃计数
    uint16_t cntFirCurB;//滤波舍弃计数

	int32_t emf_const;//反电动势常数
    uint32_t phase_sample_next[2];//下次采样时的通道
    int32_t mos_coeff;		//mos温度补偿系数 1024为1
	PID_Struct_t PID_Torque;
	PID_Struct_t PID_Flux;
	PID_Struct_t PI_BusCurLimit;				  //总线电流滤波PI
	
	int16_t hTorque_Reference;
	int16_t hFlux_Reference;
	int16_t	 IaEst;  //预测A相电流 								   /**< @f$I_{A}@f$ estimated. */
	int16_t   IbEst;  //预测B相电流								  /**< @f$I_{B}@f$ estimated. */
	int16_t   IcEst;  //预测C相电流
	volatile bool  useEstCurrent;//标记是否使用预测电流
	uint16_t ADC_BusCrtOfst;   //总线电流偏移值

 	RC_f Curr_RC_a;//
    RC_f Curr_RC_b;//
    RC_f Curr_RC_q;//
    RC_f Curr_RC_d;//
    RC_f Curr_RC_bus;//总线低通滤波器
    void *ADCx;
    void *ADC2x;
    void *MotorTIM;

	int16_t motor_bus_current;//0.1A
    volatile int16_t current_dc_bus_adc;//总线电流adc
    int16_t current_dc_bus_target;//总线电流限流adc
    su16UVW hPhaseOffset;
    volatile PWM_TIME hTime;
	uint32_t high_duty;
	uint32_t mid_duty;
	uint32_t low_duty;
    SystStatus_t State;
	 uint16_t first_open_mos_cnt;//第一次打开mos计数

    uint8_t reverse_flag;//目标正反转标记
	uint8_t bSector;
	uint8_t PWM4Direction;

    uint8_t svpwm_typde_now;//0 7段式 1 5段式
    uint8_t svpwm_typde_target;//0 7段式 1 5段式
    uint8_t  motor_lock_flag;  //下桥抱死标志
    uint8_t motor_run_flag;//油门控制
    uint8_t motor_brake_flag;//刹车控制
    
    uint8_t ebs_count;//进入下桥抱死计数
    uint8_t break_in_cnt;//触发break in计数
    uint8_t break_in_flag;//触发break in标志
    int16_t bus_vol_adc;
    uint16_t ebs_pwm_duty;
	
    PID_Struct_t PID_Speed;
    PID_Struct_t PID_Speedlimit;
	PID_Struct_t PID_Speedsync;//速度同步
	PID_Struct_t PID_Brake;   //刹车PID

    RC_f RC_Speed;//速度低通滤波器

	uint8_t theta_flag;		//角度使用标志0 hall 1 smo
    int16_t theta_angle;		//实际使用电角度
    uint8_t theta_swtich_flag;//切换角度模式
    int16_t theata_err;//保存切换时的角度误差
    int16_t theata_err_step;//切换时角度误差补偿的step
 
	uint16_t speed_loop_div;
	uint16_t speed_loop_div_max;
    int16_t motor_speed_target;		  //目标速度给定
    int16_t motor_speed_set;//0.01km/h
    int16_t motor_speed_real;       //0.01Km/H
}_FOC_CtrProc;

extern _FOC_CtrProc pFOC_ctr_handle;
extern _Hall_Obj gHall_Obj;
#endif
