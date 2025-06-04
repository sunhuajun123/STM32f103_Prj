#include "mc_hw_pwm.h"
#include "MC_Type.h"
#include "MC_Control_Param.h"
#include "stm32f10x.h"

#define ADC_PCLK_DIV    RCC_PCLK2_Div6  //120M/6=20M
#define ADC_MODE       ADC_Mode_RegInjecSimult 

void SVPWM_3ShuntInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
    TIM_OCInitTypeDef TIM1_OCInitStructure;
    TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;

    RCC_ADCCLKConfig(ADC_PCLK_DIV);//f=12.5M
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*
    ** TIM1 通道123做互补输出，用于驱动开关管通断
    ** PWM设定频率：15K；时钟频率72M
    ** 半桥驱动芯片：IR2136S,输入采用边沿触发输出状态切换，负逻辑。
    ** HI上升沿--HO输出L；HI下降沿--HO输出H；低端逻辑同高端
    */
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
    /* Time Base configuration */
    TIM1_TimeBaseStructure.TIM_Prescaler = PWM_PRSC;                        //定时器分频系数
    TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//中央对齐模式1，输出比较中断标志位只在计数器向下计数时被置位，中央对齐模式2：向上计数时被置位；中央对齐模式3：向上向下计数都置位   
    TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;                         //定时器计数周期
    TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;                //外部时钟滤波系数
    TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;                //重复计数器的值，重复溢出REP_RATE次产生溢出中断
    TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);                        //定时器初始化

    TIM_OCStructInit(&TIM1_OCInitStructure);
    /* Channel 1, 2,3 in PWM mode */
    TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                     //向上计数时，CNT<CCR时通道为有效电平，向下计数时CNT>CCR通道为无效电平
    TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;         //通道输出使能
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;       //通道互补输出使能
    TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD / 2;                       //dummy value
    TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;              
    TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;        
    TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM1_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM1_OCInitStructure);


    //设置通道4用于同步电流采样，(触发AD采样);当ADC由外部信号事件触发转换时，只有它的上升沿可以启动转换
    TIM_OCStructInit(&TIM1_OCInitStructure);
    TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  //向上计数时，CNT<CCR，通道为无效电平，向下计数，CNT>CCR,通道为有效电平
    TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - 1; 

    TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          //有效状态high
    TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       // 空闲状态low
    TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC4Init(TIM1, &TIM1_OCInitStructure);

    /* Enables the TIM1 Preload Register */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;        //运行模式下
    TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;        //空闲模式下
    TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;             //锁定级别1
    TIM1_BDTRInitStructure.TIM_DeadTime = DEADSET;                      //死区时间
    TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;   //刹车输入有效极性
    TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//自动输出失能

    TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

    TIM_SelectMasterSlaveMode(TIM1,TIM_MasterSlaveMode_Enable);//
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Reset);//
    
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    
    TIM_ClearFlag(TIM1, TIM_FLAG_Update|TIM_FLAG_COM);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM1, TIM_IT_CC4, DISABLE);

    TIM1->CCER  = 0x1888;
    
    /* ADC1 registers reset ----------------------------------------------------*/
    ADC_DeInit(ADC1);
    /* ADC2 registers reset ----------------------------------------------------*/
    ADC_DeInit(ADC2);	
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Enable ADC2 */
    ADC_Cmd(ADC2, ENABLE);
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;   //混合的同步规则+注入同步模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_DMACmd(ADC1, ENABLE);
    
    /* ADC2 Configuration ------------------------------------------------------*/
    ADC_StructInit(&ADC_InitStructure);  
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;   //混合的同步规则+注入同步模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    
    //==============================================================================
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
    /* Enable ADC2 */
    ADC_Cmd(ADC2, ENABLE);   //唤醒ADC2
    /* Enable ADC2 reset calibaration register */   
    ADC_ResetCalibration(ADC2);
    /* Check the end of ADC2 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC2));
    /* Start ADC2 calibaration */
    ADC_StartCalibration(ADC2);
    /* Check the end of ADC2 calibration */
    while(ADC_GetCalibrationStatus(ADC2));
    //================================================================================== 
    
    uint32_t NvicPrio_Group = NVIC_GetPriorityGrouping();

    /* Enable the ADC Interrupt */
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NvicPrio_Group, 1, 0));
    NVIC_EnableIRQ(ADC1_2_IRQn);
    
    /* Enable the Update Interrupt */
    NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NvicPrio_Group, 1, 0));
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    /* Enable the TIM1 BRK Interrupt */
    NVIC_SetPriority(TIM1_BRK_IRQn, NVIC_EncodePriority(NvicPrio_Group, 0, 0));
    NVIC_EnableIRQ(TIM1_BRK_IRQn);
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}