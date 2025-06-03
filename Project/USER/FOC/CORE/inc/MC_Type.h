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
	M1 = 0,//�����
	M2 = 1,//�ӵ��
}_MOTOR_NUM;//����ĸ����

typedef enum
{
    INIT = 0,       //��ʼ��
    IDLE = 1,
    STOP = 2,
    WAIT = 3,
    FAULT = 4,
    ALIGNED = 5,    //����
    OPENLOOP = 6,   //��������
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

typedef struct 
{
    _MOTOR_NUM motor_num;
//	_MOTOR_RUN_MODE run_mode;

//	const u16ADC_OFFSET ADC_inidat_dft;
    ab_t Stat_Curr_ab_lowpass;
    dq_t Stat_Curr_qd_r_r;   //�������ĸ���ֵ�����ڵ�����Id,Iq��ǰ���������Ƶĸ���ֵ
    ab_t Stat_Curr_ab;
    alphabeta_t Stat_Curr_alfa_beta;
    dq_t Stat_Curr_qd;
    dq_t Stat_Curr_qd_lowpass;
    dq_t Stat_Volt_qd;
    alphabeta_t Stat_Volt_alfa_beta;
    Trig_Components Vector;
    uint16_t cntFirCurA;//�˲���������
    uint16_t cntFirCurB;//�˲���������

	int32_t emf_const;//���綯�Ƴ���
    uint32_t phase_sample_next[2];//�´β���ʱ��ͨ��
    int32_t mos_coeff;		//mos�¶Ȳ���ϵ�� 1024Ϊ1
	PID_Struct_t PID_Torque;
	PID_Struct_t PID_Flux;
	PID_Struct_t PI_BusCurLimit;				  //���ߵ����˲�PI
	
	int16_t hTorque_Reference;
	int16_t hFlux_Reference;
	int16_t	 IaEst;  //Ԥ��A����� 								   /**< @f$I_{A}@f$ estimated. */
	int16_t   IbEst;  //Ԥ��B�����								  /**< @f$I_{B}@f$ estimated. */
	int16_t   IcEst;  //Ԥ��C�����
	volatile bool  useEstCurrent;//����Ƿ�ʹ��Ԥ�����
	uint16_t ADC_BusCrtOfst;   //���ߵ���ƫ��ֵ

 	RC_f Curr_RC_a;//
    RC_f Curr_RC_b;//
    RC_f Curr_RC_q;//
    RC_f Curr_RC_d;//
    RC_f Curr_RC_bus;//���ߵ�ͨ�˲���
    void *ADCx;
    void *ADC2x;
    void *MotorTIM;

	int16_t motor_bus_current;//0.1A
    volatile int16_t current_dc_bus_adc;//���ߵ���adc
    int16_t current_dc_bus_target;//���ߵ�������adc
    su16UVW hPhaseOffset;
    volatile PWM_TIME hTime;
	uint32_t high_duty;
	uint32_t mid_duty;
	uint32_t low_duty;
    SystStatus_t State;
	 uint16_t first_open_mos_cnt;//��һ�δ�mos����

    uint8_t reverse_flag;//Ŀ������ת���
	uint8_t bSector;
	uint8_t PWM4Direction;

    uint8_t svpwm_typde_now;//0 7��ʽ 1 5��ʽ
    uint8_t svpwm_typde_target;//0 7��ʽ 1 5��ʽ
    uint8_t  motor_lock_flag;  //���ű�����־
    uint8_t motor_run_flag;//���ſ���
    uint8_t motor_brake_flag;//ɲ������
    
    uint8_t ebs_count;//�������ű�������
    uint8_t break_in_cnt;//����break in����
    uint8_t break_in_flag;//����break in��־
    int16_t bus_vol_adc;
    uint16_t ebs_pwm_duty;
	
    PID_Struct_t PID_Speed;
    PID_Struct_t PID_Speedlimit;
	PID_Struct_t PID_Speedsync;//�ٶ�ͬ��
	PID_Struct_t PID_Brake;   //ɲ��PID

    RC_f RC_Speed;//�ٶȵ�ͨ�˲���

	uint8_t theta_flag;		//�Ƕ�ʹ�ñ�־0 hall 1 smo
    int16_t theta_angle;		//ʵ��ʹ�õ�Ƕ�
    uint8_t theta_swtich_flag;//�л��Ƕ�ģʽ
    int16_t theata_err;//�����л�ʱ�ĽǶ����
    int16_t theata_err_step;//�л�ʱ�Ƕ�������step
 
	uint16_t speed_loop_div;
	uint16_t speed_loop_div_max;
    int16_t motor_speed_target;		  //Ŀ���ٶȸ���
    int16_t motor_speed_set;//0.01km/h
    int16_t motor_speed_real;       //0.01Km/H
}_FOC_CtrProc;

extern _FOC_CtrProc pFOC_ctr_handle;
extern _Hall_Obj gHall_Obj;
#endif
