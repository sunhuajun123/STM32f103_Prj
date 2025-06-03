#ifndef __MC_CONTROL_PARAM_H__
#define __MC_CONTROL_PARAM_H__

#define CKTIM    (72000000uL)

#define PWM_FREQ (15000ul) // in Hz  (N.b.: pattern type is center aligned)
#define REP_RATE (1)  // (N.b): Internal current loop is performed every 
                      //             (REP_RATE + 1)/(2*PWM_FREQ) seconds.
                      // REP_RATE has to be an odd number in case of three-shunt
                      // current reading; this limitation doesn't apply to ICS
#define SAMPLING_FREQ ((uint16_t)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

/****    Deadtime Value   默认时间500***/
#define DEADTIME_NS	(800ul)  //in nsec; range is [0...4000] 最大32000ns

//PWM 时钟分频
#define PWM_PRSC (0ul)
/* Resolution: 2000 用于定时器载入 */                            
#define PWM_PERIOD      ((uint16_t)(CKTIM / (uint32_t)(2 * PWM_FREQ *(PWM_PRSC+1)))) 


#define DEADTIME       (CKTIM/2/1000000 * DEADTIME_NS/1000uL) 
 //用于死区时间的设置,精度与分频有关：Dts=CKTIM/DIV=31.25

#if (DEADTIME > 512ul)&&(DEADTIME <= 1024ul)//512Dts~1024Dts;16000-32000ns;步长500ns
#define DEADSET         (uint8_t)(0xE0|((uint16_t)(DEADTIME-512)/16))
#elif (DEADTIME > 256ul)//256Dts~512Dts;8000-16000ns;步长250ns
#define DEADSET         (uint8_t)(0xC0|((uint16_t)(DEADTIME-256)/8))
#elif (DEADTIME > 128ul)//128Dts~256Dts;4000-8000ns;步长62.5ns
#define DEADSET         (uint8_t)(0x80|((uint16_t)(DEADTIME-128)/2))
#elif (DEADTIME <= 128ul)//0~128Dts；0--4000ns;步长31.25ns
#define DEADSET         (uint8_t)DEADTIME
#else 
#error "deadtime wrong"
#endif




#endif
