#include "timer.h"
#include "led.h"
#include "exti.h"    
#include "delay.h"
#include "usart.h"   
#include "response.h"
#include <string.h>
#include "modbus_slave.h"

volatile  unsigned char One_second =0;   

volatile  unsigned int IC2Value =0;   

volatile  float Frequency =0;   
volatile  float DutyCycle =0; 

//通用定时器2中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器2
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
    
    //定时器TIM2初始化
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

    TIM_Cmd(TIM2, ENABLE);  //使能TIMx                     
}

//定时器2中断服务程序
void TIM2_IRQHandler(void)   //TIM2中断
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志 
        LED0=!LED0;
        //One_second =1 ;     
        ///printf("beta_cnt = %d \r\n",beta_cnt);   
			RS485_Service();
    }
}

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器5
void TIM3_Int_Init(u16 arr,u16 psc)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
    
    //定时器TIM3初始化
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级2级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器           
        
}

//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
        TIM_Cmd(TIM3, DISABLE);  //使能TIMx    
         
        memcpy(Receive_Buffer,USART_RX_BUF,USART_REC_LEN);
        memset(USART_RX_BUF, 0, USART_REC_LEN );
        CheackDat(Receive_Buffer);
    }
}

//void Tim3_PWMIC_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);      //时钟配置
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_7;                               //GPIO配置
//	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel= TIM3_IRQn;                     //NVIC配置
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;     //通道选择
//	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
//	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
//	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频.意思是控制在
//	//多少个输入周期做一次捕获,如果
//	//输入的信号频率没有变,测得的周期也不会变.比如选择4分频,则每四个输入周期才做一次捕获
//	//这样在输入信号变化不频繁的情况下,可以减少软件不被中断的次数
//	TIM_ICInitStructure.TIM_ICFilter= 0x0;                            //滤波设置,经历几个周期跳变认定波形稳定0x0~0xF
//	TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);                 //根据参数配置TIM外设信息
//	TIM_SelectInputTrigger(TIM3,TIM_TS_TI2FP2);                //选择IC2为时钟触发源
//	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);//TIM从模式,触发信号的上升沿重新初始化计数器和触发寄存器更新事件
//	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable); //启动定时器的被动触发
//	TIM_Cmd(TIM3,ENABLE);                                 //启动TIM3
//	TIM_ITConfig(TIM3,TIM_IT_CC2, ENABLE);     //打开中断 
//}


////中断服务函数
//void TIM3_IRQHandler(void) 
//{
//	TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);                //清除TIM的中断待处理位
//  IC2Value =TIM_GetCapture2(TIM3);                         //读取IC2捕获寄存器的值,即为PWM周期的计数值
//  if (IC2Value != 0)
//  {   
//    DutyCycle= (TIM_GetCapture1(TIM3) * 100) / IC2Value;         //读取IC1捕获寄存器的值,并计算占空比
//       Frequency= 72000000 / IC2Value;                                          //计算PWM频率
//  }
//  else
//  {
//    DutyCycle= 0;
//    Frequency= 0;
//  }
//}

