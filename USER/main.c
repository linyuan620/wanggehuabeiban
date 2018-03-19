#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "led.h"
#include "spi.h"
#include "math.h"
#include "am2305.h"
#include "fsx2.h"
#include "adc.h"
#include "rs485.h"
#include "awa5636.h"
#include <string.h>
#include <stdlib.h>
#include "modbus_slave.h"

//全局变量
//AD采样存放空间 
__IO uint16_t ADCConvertedValue[20]; 

#define N 10 //每个通道采集10次
#define M 3 //总共3个通道

u16 adcx;
u16 adcy;
float voltage_vs;
float voltage_mpx;
float voltage_pwm;

char* sendToDvr = NULL;
char* sendStart="Start<";
char* s_windSpeed="WindSpeed:";
char* s_windDir=";WindDir:";
char* s_temperature=";Temperature:";
char* s_humidity=";Humidity:";
char* s_pressure=";Pressure:";
char* s_tsp=";Tsp:";
char* s_pm10=";Pm10:";
char* s_pm25=";Pm25:";
char* s_noise=";Noise:";
char* sendEnd=";>End";
char* s_temp = NULL;

char* s_co=";co:";
char* s_no2=";no2:";
char* s_so2=";so2:";
char* s_o3=";o3:";
char* s_tvoc=";tvoc:";

u16 tempHumidity;
u16 tempTemperature; 
u16 pressure;

extern u16 windSpeedDirType; //400001
extern u16 noiseType;  //400002
extern u16 laserType;   //400003

void Send_All_Result(void);
void Refresh_Led_Scream(void);
//函数
//初始化AD  
void init_ad(void)  
{  
    ADC_InitTypeDef ADC_InitStructure;  
    DMA_InitTypeDef DMA_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;  
	
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
    DMA_DeInit(DMA1_Channel1);  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
    DMA_InitStructure.DMA_BufferSize = M*N;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;   
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);  

    DMA_Cmd(DMA1_Channel1, ENABLE);  

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;  
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  
    ADC_InitStructure.ADC_NbrOfChannel = M;  
    ADC_Init(ADC1, &ADC_InitStructure);  
       
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4 , 1, ADC_SampleTime_239Cycles5);  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12 , 2, ADC_SampleTime_239Cycles5);  
		ADC_RegularChannelConfig(ADC1, ADC_Channel_7 , 3, ADC_SampleTime_239Cycles5);  

    ADC_DMACmd(ADC1, ENABLE);  

    ADC_Cmd(ADC1, ENABLE);  
  
    ADC_ResetCalibration(ADC1);  
    while(ADC_GetResetCalibrationStatus(ADC1));  

    ADC_StartCalibration(ADC1);  

    while(ADC_GetCalibrationStatus(ADC1));  
 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);  
}  
  

float get_voltage_vs(void)  
{  
    uint8_t i = 0;  
    uint16_t sum = 0;  
    float v = 0;  
   
    for (i = 0;i < 10;i++)  
    {  
        sum += ADCConvertedValue[i * M];   
    }  
    sum /= 10;  
    v = 2*(3.0/4096) * sum;  
  
    return v;  
}  
   
float get_voltage_mpx(void)  
{  
    uint8_t i = 0;  
    uint16_t sum = 0;  
    float v = 0;  

    for (i = 0;i < 10;i++)  
    {  
        sum += ADCConvertedValue[i * M + 1];   
    }  
    sum /= 10;  
    v = 2*(3.0/4096) * sum;  
    return v;  
}  

float get_voltage_pwm(void)  
{  
    uint8_t i = 0;  
    uint16_t sum = 0;  
    float v = 0;  
   
    for (i = 0;i < 10;i++)  
    {  
        sum += ADCConvertedValue[i * M + 2];   
    }  
    sum /= 10;  
    v = (3.0/4096) * sum;  
 
    return v;  
} 


int main(void)
{		
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	LED_Init();			     //LED端口初始化
	//uart1_init(9600);	 	//串口初始化为9600  wifi
	TIM2_Int_Init(49,7199);//10Khz的计数频率，计数到50为5ms  //50ms计时  led_run modbus 处理函数
	//Tim3_PWMIC_Init();
	//AM2305_Init(); 
	
	//Adc_Init(); 	
	init_ad();
	RS485_Init(9600); //usart3 fsx tsp nvr 485  作为modbus主机读取传感器数据  不用定时器 靠延迟约50ms接收
	//uart4_init(9600);//uart4 ble ttl 本用于主动向工控机主动上传
	RS485_Initt(9600);//uart4 modbus从机
	
	while(1)
	{	                                                           
		ReadRsTemHum();
	
		switch(windSpeedDirType){
			//这里和上版本对调 1 和 2
			case 0:
				//一体风速风向传感器
				ReadWindSpeedDir();
				break;
			case 2:
				//分体风速风向传感器	
				ReadWindSpeed ();
				ReadWindDir();
				break;
			case 1:
				//超生波 富奥通
				ReadUltraSound();
				break;
			default:
				
				break;
		}
		
		switch(laserType){
			case 0:
				break;
			case 1:
				//读取苏静与攀藤pm值
				ReadLaserTsp();
				break;
			case 2:
				break;
			default:
				break;
		}
		
		ReadAllToxicGas();
		
		//大气压力采集
		
		voltage_vs = get_voltage_vs();
		voltage_mpx = get_voltage_mpx();
		
		pressure = 10 * ( ( ( (get_voltage_mpx() - 0.07736)/get_voltage_vs() ) + 0.1518 ) / 0.01059 );
		
		switch(noiseType){
			case 0:
				break;
			case 1:
				//噪声采集 爱华dc
					noise = ( get_voltage_pwm() *1000/20 ) * 10;
				break;
			case 2:
				break;
			default:
				break;
		}
		
		Refresh_Led_Scream();
		//Send_All_Result();	
		//delay_ms(200);//要延迟否则接受数据中断
	}
}

void Refresh_Led_Scream(void){
	SendLedScreamCmd();
}

void Send_All_Result(void)
{
	u8 t=0;
	
	sendToDvr = (char *)malloc(300);
	memset(sendToDvr,0,300);
	s_temp = (char *)malloc(20);
	memset(s_temp,0,20);
	
	strcat(sendToDvr,sendStart);	
	strcat(sendToDvr,s_windSpeed);
	sprintf(s_temp,"%d",windSpeed);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_windDir);
	sprintf(s_temp,"%d",windDir);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_temperature);
	sprintf(s_temp,"%d",OutdoorTemperature);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_humidity);
	sprintf(s_temp,"%d",OutdoorHumidity);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_pressure);
	sprintf(s_temp,"%d",pressure);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_tsp);
	sprintf(s_temp,"%d",LaserTsp);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_pm10);
	sprintf(s_temp,"%d",LaserPm10);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_pm25);
	sprintf(s_temp,"%d",LaserPm25);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_noise);
	sprintf(s_temp,"%d",noise);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_co);
	sprintf(s_temp,"%d",CoPpm);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_no2);
	sprintf(s_temp,"%d",No2Ppm);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_so2);
	sprintf(s_temp,"%d",So2Ppm);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_o3);
	sprintf(s_temp,"%d",O3Ppm);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,s_tvoc);
	sprintf(s_temp,"%d",TVOCPpm);
	strcat(sendToDvr,s_temp);
	
	strcat(sendToDvr,sendEnd);
	//发送数据到DVR
//	RS485_TX_EN=1;			//设置为发送模式
//	for(t=0;t<strlen(sendToDvr);t++)		//循环发送数据
//	{		   
//		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	  
//		USART_SendData(USART3,sendToDvr[t]);
//	}	 
//	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
 //发送数据到BLE 或者 背板
	for(t=0;t<strlen(sendToDvr);t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART4,sendToDvr[t]);
	}	 
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
	
	free(s_temp);
	free(sendToDvr);
}




//模组板
//usart1 modbus从机 3 0 
//time3 5ms modbus 帧截取定时器 3 1
//time2 5s 中断 led modbus服务处理 3 2


//工控机背板
//主优先级 从优先级
//uart4 3 0     //modbus从机
//time7 3 1     // 帧截取定时器
//time2 3 2     // 50ms led 与 modbus服务处理
//usart3 3 3   //modbus主机
