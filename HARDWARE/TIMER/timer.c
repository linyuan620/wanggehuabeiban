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

//ͨ�ö�ʱ��2�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��2
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
    
    //��ʱ��TIM2��ʼ��
    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

    TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx                     
}

//��ʱ��2�жϷ������
void TIM2_IRQHandler(void)   //TIM2�ж�
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
        LED0=!LED0;
        //One_second =1 ;     
        ///printf("beta_cnt = %d \r\n",beta_cnt);   
			RS485_Service();
    }
}

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��5
void TIM3_Int_Init(u16 arr,u16 psc)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
    
    //��ʱ��TIM3��ʼ��
    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ   
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���           
        
}

//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ 
        TIM_Cmd(TIM3, DISABLE);  //ʹ��TIMx    
         
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

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);      //ʱ������
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
//	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_7;                               //GPIO����
//	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA,&GPIO_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel= TIM3_IRQn;                     //NVIC����
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;     //ͨ��ѡ��
//	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
//	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
//	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ.��˼�ǿ�����
//	//���ٸ�����������һ�β���,���
//	//������ź�Ƶ��û�б�,��õ�����Ҳ�����.����ѡ��4��Ƶ,��ÿ�ĸ��������ڲ���һ�β���
//	//�����������źű仯��Ƶ���������,���Լ�����������жϵĴ���
//	TIM_ICInitStructure.TIM_ICFilter= 0x0;                            //�˲�����,�����������������϶������ȶ�0x0~0xF
//	TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);                 //���ݲ�������TIM������Ϣ
//	TIM_SelectInputTrigger(TIM3,TIM_TS_TI2FP2);                //ѡ��IC2Ϊʱ�Ӵ���Դ
//	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);//TIM��ģʽ,�����źŵ����������³�ʼ���������ʹ����Ĵ��������¼�
//	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable); //������ʱ���ı�������
//	TIM_Cmd(TIM3,ENABLE);                                 //����TIM3
//	TIM_ITConfig(TIM3,TIM_IT_CC2, ENABLE);     //���ж� 
//}


////�жϷ�����
//void TIM3_IRQHandler(void) 
//{
//	TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);                //���TIM���жϴ�����λ
//  IC2Value =TIM_GetCapture2(TIM3);                         //��ȡIC2����Ĵ�����ֵ,��ΪPWM���ڵļ���ֵ
//  if (IC2Value != 0)
//  {   
//    DutyCycle= (TIM_GetCapture1(TIM3) * 100) / IC2Value;         //��ȡIC1����Ĵ�����ֵ,������ռ�ձ�
//       Frequency= 72000000 / IC2Value;                                          //����PWMƵ��
//  }
//  else
//  {
//    DutyCycle= 0;
//    Frequency= 0;
//  }
//}

