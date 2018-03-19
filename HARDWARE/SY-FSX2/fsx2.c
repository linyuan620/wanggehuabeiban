#include "fsx2.h"
#include "delay.h"
#include "usart.h"	 
#include "rs485.h"
#include <stdlib.h>
#include "am2305.h"
#include "mbcrc.h"

//读取矿山粉尘仪数据
// 0x55, 0x01, 0x00, 0x00, 0xf2

//SY-FSX2 风速风向一体
//读取风速 只读 地址0x002a 42
// 0x02 0x03 0x00 0x2a 0x00 0x01 0xa5 0xf1
// 0x02 0x03 0x02 0x00 0x00 0xfc 0x44
//读取风向 只读 地址0x002b 43
// 0x02 0x03 0x00 0x2b 0x00 0x01 0xf4 0x31
// 0x02 0x03 0x02 0x03 0xb4 0xfc 0xd7

//读取风速风向
//0x02 0x03 0x00 0x2A 0x00 0x02 0xE5 0xF0

//设备地址 可读可写 地址0x2000 8190
//0x02 0x03 0x20 0x00 0x00 0x01 0x8F 0xF9
//0x02 0x03 0x02 0x00 0x02 0x7D 0x85

//u8 FSX_TX_BUF[8]={0x02,0x03,0x00,0x2a,0x00,0x01,0xa5,0xf1}; 

//读取易谷风速传感器的值
//风速传感器地址设为2
//00 10 01 02 81 b0
//返回
//00 10 01 01 C1 B1
//第四位01为成功
//读取风速值 地址为00
//Tx:02 03 00 00 00 01 84 39
//Rx:02 03 02 00 12 7C 49

//读取易谷风向传感器的值
//风向传感器地址设为3
//00 10 01 03 40 70
//返回
//00 10 01 01 C1 B1
//第四位01为成功
//读取风速值 地址为00
//Tx: 03 03 00 00 00 01 85 E8
//Rx: 03 03 02 01 29 01 CA

#define  FLITE_NUM 60
#define 	true 1
#define 	false 0
#define 	ErrorMax 10

int tmd;

unsigned char WindErrorCnt = 0;
unsigned char TemHumErrorCnt = 0;
unsigned char PmErrorCnt = 0;

unsigned char CoErrorCnt = 0;
unsigned char No2ErrorCnt = 0;
unsigned char So2ErrorCnt = 0;
unsigned char O3ErrorCnt = 0;
unsigned char H2sErrorCnt = 0;
unsigned char TvocErrorCnt = 0;

u16 OutdoorHumidityTem =0;
s16 OutdoorTemperatureTem = 0;

//一体风速风向 02
u8 FSX_TX_BUF[8]={0x02,0x03,0x00,0x2A,0x00,0x02,0xE5,0xF0}; 
//分体风速 地址 02
u8 FS_TX_BUF[8]={0x02,0x03,0x00,0x00,0x00,0x01,0x84,0x39}; 
//分体风向 地址 03
u8 FX_TX_BUF[8]={0x03,0x03,0x00,0x00,0x00,0x01,0x85,0xE8}; 

u8 YLY_TX_BUF[43] ={0x20,0x10,0x00,0x00,0x00,0x11,0x22,0x00,0x01,0x00 
									 ,0x02,0x00,0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00
									 ,0x46,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x64,0x00
									 ,0x00,0x00,0x6E,0x00,0x00,0x00,0x78,0x00,0x00,0x00
									 ,0x82,0xC2,0x1C};
	
//富奥通超声波风速风向 地址48
u8 USX_TX_BUF[8]={0x30,0x03,0x00,0x00,0x00,0x05,0x81,0xE8};

//485温湿度 地址04
u8 TH_TX_BUF[8]={0x04,0x03,0x00,0x00,0x00,0x02,0xC4,0x5E}; 
//矿山粉尘仪
u8 CLD_TX_BUF[5]={0x55,0x01,0x00,0x00,0xF2}; 
//读取Pm 
u8 LD_TX_BUF[8]={0x01,0x04,0x00,0x06,0x00,0x03,0x50,0x0a}; 

//co 地址 01
u8 CO_TX_BUF[11]  = {0xFF,0xFF,0x01,0x01,0x05,0x01,0x00,0x6C,0x07,0x74,0xDD};
//no2 地址 02
u8 NO2_TX_BUF[11] = {0xFF,0xFF,0x01,0x02,0x05,0x01,0x00,0x6C,0x07,0x74,0xDD};
//so2 地址 03
u8 SO2_TX_BUF[11] = {0xFF,0xFF,0x01,0x03,0x05,0x01,0x00,0x6C,0x07,0x74,0xDD};
//o3 地址 04
u8 O3_TX_BUF[11]  = {0xFF,0xFF,0x01,0x04,0x05,0x01,0x00,0x6C,0x07,0x74,0xDD};
//预留 地址 05 H2s
u8 H2S_TX_BUF[11]  = {0xFF,0xFF,0x01,0x05,0x05,0x01,0x00,0x6C,0x07,0x74,0xDD};
//tvoc modbus 地址
u8 TVOC_TX_BUF[8]  = {0x01,0x04,0x00,0x14,0x00,0x02,0x31,0xcf};

u8 FSX_RX_BUF[64]={0}; 

u8 TH_RX_BUF[64]={0}; 

u8 USX_RX_BUF[64]={0}; 

u8 CLD_RX_BUF[64]={0};

u8 LD_RX_BUF[64]={0};

u8 CO_RX_BUF[64]={0};

u8 NO2_RX_BUF[64]={0};

u8 SO2_RX_BUF[64]={0};

u8 O3_RX_BUF[64]={0};

u8 H2S_RX_BUF[64]={0};

u8 TVOC_RX_BUF[64]={0};

u16 windSpeed=0;
u16 windDir=0;

u16 LaserPm10 = 0;

u16 LaserTsp = 0;
u16 LaserPm25 = 0;

u32 CoPpm = 0;
u32 No2Ppm = 0;
u32 So2Ppm = 0;
u32 O3Ppm = 0;
u32 TVOCPpm = 0;
u32 H2sPpm = 0;

u32 CoPpmBuf[64]={0};
u32 No2PpmBuf[64]={0};
u32 So2PpmBuf[64]={0};
u32 O3PpmBuf[64]={0};
u32 H2sPpmBuf[64]={0};
u32 TVOCPpmBuf[64]={0};	
	
u8 CoCnt = 0;
u8 No2Cnt = 0;
u8 So2Cnt = 0;
u8 O3Cnt = 0;
u8 H2sCnt = 0;
u8 TVOCCnt = 0;
	
u16 randSeed = 0;
float randK = 0;

#define TSP_K  2.65
#define PM25_K 0.65

extern u16 pressure;//300005

u32 filtStack(u32* source,u32 readFiltBuf[],u8* readCnt,u8 isRand);

int ReadWindSpeedDir(void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=FSX_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(FSX_RX_BUF,&revLen);//读取九个数据
	if(revLen == 9)
	{
		calCRC = usMBCRC16(FSX_RX_BUF,revLen-2);
		recCRC = FSX_RX_BUF[revLen-2]|(((u16)FSX_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			windSpeed = (FSX_RX_BUF[3]<<8)|FSX_RX_BUF[4];
			windDir = (FSX_RX_BUF[5]<<8)|FSX_RX_BUF[6];
			WindErrorCnt = 0;
			return true;
		}
	}
	if(WindErrorCnt++ > ErrorMax){
	WindErrorCnt = ErrorMax;	
	windSpeed = windDir = 0;
	}
	return false;
}

int ReadUltraSound(void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=USX_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(USX_RX_BUF,&revLen);//读取九个数据
	if(revLen == 15)
	{
		calCRC = usMBCRC16(USX_RX_BUF,revLen-2);
		recCRC = USX_RX_BUF[revLen-2]|(((u16)USX_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			windSpeed = (USX_RX_BUF[11]<<8)|USX_RX_BUF[12];//风速传感器传回的已经乘以10
			windDir = 10* (USX_RX_BUF[5]<<8)|USX_RX_BUF[6];//风向传感器传回来的是原始整数
			//上传的时候都要扩大10倍
			WindErrorCnt = 0;
			return true;
		}
	}
	if(WindErrorCnt++ > ErrorMax){
	WindErrorCnt = ErrorMax;	
	windSpeed = windDir = 0;
	}
	return false;
}

//刷新内容指令 波特率 9600 地址 0x20
//Tx:000-20 10 00 00 00 11 22 00 01 00 02 00 03 00 04 00 05 00 06 00 46 00 50 00 00 00 00 00 64 00 00 00 6E 00 00 00 78 00 00 00 82 C2 1C
//解析:
//Tx:
//20 10                  0 1  
//00 00    2 3
//00 11 22  4 5 6 
//00 01   风速一个小数点   1   7 8 
//00 02   风向一个小数点   2   9 10
//00 03   温度一个小数点   3   11 12
//00 04   湿度一个小数点   4   13 14
//00 05   压力一个小数点   5   15 16
//00 06   6   17 18
//00 46    PM10 一个小数点 7 19 20
//00 50    Pm2.5 一个小数点 8   21 22
//00 00   9                             23 24
//00 00 00 64    CO 无小数点 10 11      25 26 27 28
//00 00 00 6E    NO2 无小数点 12 13     29 30 31 32
//00 00 00 78    SO2 无小数点  14 15    33 34 35 36
//00 00 00 82    O3  无小数点    16 17  37 38 39 40
//C2 1C                          41 42 

void SendLedScreamCmd (void)
{
	u16 calCRC;
	unsigned char i=0;
	//unsigned char revLen=0;
	unsigned char  rs485buf[64];
	
	{
		YLY_TX_BUF[7] = windSpeed>>8;
		YLY_TX_BUF[8] = windSpeed;
		YLY_TX_BUF[9] = windDir>>8;
		YLY_TX_BUF[10] = windDir;
		YLY_TX_BUF[11] = OutdoorTemperature>>8;
		YLY_TX_BUF[12] = OutdoorTemperature;
		YLY_TX_BUF[13] = OutdoorHumidity>>8;
		YLY_TX_BUF[14] = OutdoorHumidity;
		YLY_TX_BUF[15] = pressure>>8;
		YLY_TX_BUF[16] = pressure;
		
		YLY_TX_BUF[19] = LaserPm10>>8;
		YLY_TX_BUF[20] = LaserPm10;

		YLY_TX_BUF[21] = LaserPm25>>8;
		YLY_TX_BUF[22] = LaserPm25;
		
		YLY_TX_BUF[25] = CoPpm>>24;
		YLY_TX_BUF[26] = CoPpm>>16;
		YLY_TX_BUF[27] = CoPpm>>8;
		YLY_TX_BUF[28] = CoPpm;
		
		YLY_TX_BUF[29] = No2Ppm>>24;
		YLY_TX_BUF[30] = No2Ppm>>16;
		YLY_TX_BUF[31] = No2Ppm>>8;
		YLY_TX_BUF[32] = No2Ppm;
		
		YLY_TX_BUF[33] = So2Ppm>>24;
		YLY_TX_BUF[34] = So2Ppm>>16;
		YLY_TX_BUF[35] = So2Ppm>>8;
		YLY_TX_BUF[36] = So2Ppm;
		
		YLY_TX_BUF[37] = O3Ppm>>24;
		YLY_TX_BUF[38] = O3Ppm>>16;
		YLY_TX_BUF[39] = O3Ppm>>8;
		YLY_TX_BUF[40] = O3Ppm;
		
		calCRC = usMBCRC16(YLY_TX_BUF,41);;
		
		YLY_TX_BUF[42] = calCRC>>8;
		YLY_TX_BUF[41] = calCRC;

	}
	
	for(i=0;i<43;i++)
	{
		rs485buf[i]=YLY_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,43);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
}

int ReadWindSpeed (void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=FS_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(FSX_RX_BUF,&revLen);//读取七个数据
	if(revLen == 7)
	{
		calCRC = usMBCRC16(FSX_RX_BUF,revLen-2);
		recCRC = FSX_RX_BUF[revLen-2]|(((u16)FSX_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			windSpeed =(FSX_RX_BUF[3]<<8)|FSX_RX_BUF[4];	
			WindErrorCnt = 0;
			return true;
		}			
	}
	if(WindErrorCnt++ > ErrorMax){
	WindErrorCnt = ErrorMax;	
	windSpeed = 0;
	}
	return false;
}

int ReadWindDir(void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=FX_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(FSX_RX_BUF,&revLen);//读取七个数据
	if(revLen == 7)
	{
		calCRC = usMBCRC16(FSX_RX_BUF,revLen-2);
		recCRC = FSX_RX_BUF[revLen-2]|(((u16)FSX_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			windDir =10* ( (FSX_RX_BUF[3]<<8)|FSX_RX_BUF[4] );
			WindErrorCnt = 0;
			return true;
		}
	}
	if(WindErrorCnt++ > ErrorMax){
	WindErrorCnt = ErrorMax;	
	windDir = 0;
	}
	return false;
}

int ReadRsTemHum(void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	
	for(i=0;i<8;i++)
	{
		rs485buf[i]=TH_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(TH_RX_BUF,&revLen);//读取九个数据
	if(9 == revLen)
	{
		calCRC = usMBCRC16(TH_RX_BUF,revLen-2);
		recCRC = TH_RX_BUF[revLen-2]|(((u16)TH_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			//传感器返回*10的值
				OutdoorHumidityTem = (TH_RX_BUF[3]<<8)|TH_RX_BUF[4];
				OutdoorTemperatureTem = (s16) ((TH_RX_BUF[5]<<8)|TH_RX_BUF[6]);
			
				if( (OutdoorHumidityTem < 1000) && ( (OutdoorTemperatureTem> -200)&&(OutdoorTemperatureTem< 800)) )
				{
					OutdoorHumidity = (TH_RX_BUF[3]<<8)|TH_RX_BUF[4];
					OutdoorTemperature = (TH_RX_BUF[5]<<8)|TH_RX_BUF[6];
				
					TemHumErrorCnt = 0;
					return true;
				}
		}
	}
	if(TemHumErrorCnt++ > ErrorMax){
		TemHumErrorCnt = ErrorMax;	
		OutdoorHumidity = OutdoorTemperature = 0;
	}
	return false;
}

//读取苏静与攀藤pm值
int ReadLaserTsp(void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=LD_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(LD_RX_BUF,&revLen); 
	if(revLen == 11)
	{
		calCRC = usMBCRC16(LD_RX_BUF,revLen-2);
		recCRC = LD_RX_BUF[revLen-2]|(((u16)LD_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
//		LaserPm10 = (LD_RX_BUF[3]<<24)|(LD_RX_BUF[4]<<16)|(LD_RX_BUF[5]<<8)|LD_RX_BUF[6];
//		LaserPm10 = 10*LaserPm10;
//		
//		randSeed = rand();
//		randK  = (1+((float)randSeed-32767)/327670);
//		LaserTsp = LaserPm10 * TSP_K * randK;
//		
//		randSeed = rand();
//		randK  = (1+((float)randSeed-32767)/327670);
		
		//LaserPm25 = LaserPm10 * PM25_K * randK;
		
//		LaserTsp = (LD_RX_BUF[3]<<24)|(LD_RX_BUF[4]<<16)|(LD_RX_BUF[5]<<8)|LD_RX_BUF[6];
//		LaserTsp = 20*LaserTsp;
			
			LaserTsp =  ( (LD_RX_BUF[3]<<8)|LD_RX_BUF[4] );
			LaserPm10 =  ( (LD_RX_BUF[5]<<8)|LD_RX_BUF[6] );
			LaserPm25 =  ( (LD_RX_BUF[7]<<8)|LD_RX_BUF[8] );
			PmErrorCnt = 0;
			return true;
		}
	}
	if(PmErrorCnt++ >ErrorMax){
	LaserPm25 = LaserPm10 = 0;
	PmErrorCnt = ErrorMax;	
	}
	return false;
}

int ReadTvocPpm (void)
{
	u16 calCRC;
	u16 recCRC;
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	for(i=0;i<8;i++)
	{
		rs485buf[i]=TVOC_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,8);//发送8个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(TVOC_RX_BUF,&revLen); 
	if(revLen == 9)
	{
		calCRC = usMBCRC16(TVOC_RX_BUF,revLen-2);
		recCRC = TVOC_RX_BUF[revLen-2]|(((u16)TVOC_RX_BUF[revLen-1])<<8);
		if(calCRC == recCRC)
		{
			TVOCPpm =  ( (TVOC_RX_BUF[3]<<24)| (TVOC_RX_BUF[4]<<16) | (TVOC_RX_BUF[5]<<8)|TVOC_RX_BUF[6] );
			TvocErrorCnt = 0;
			return true;
		}
	}
	if(TvocErrorCnt++ >ErrorMax){
	TVOCPpm = 0;
	TvocErrorCnt = ErrorMax;	
	}
	return false;
}

int ReadCoPpm (void)
{
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	unsigned char cTemp =0;
	u32 CoPpmTmp = 0;
	for(i=0;i<11;i++)
	{
		rs485buf[i]=CO_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,11);//发送11个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(CO_RX_BUF,&revLen);//读取十八个数据
	if(revLen == 18)
	{
		CoPpmTmp = (CO_RX_BUF[10]<<16)|(CO_RX_BUF[11]<<8)|CO_RX_BUF[12];	 
		
		CoPpmTmp = (CoPpmTmp&0x0F)+10*((CoPpmTmp>>4)&0x0F)+\
		100*((CoPpmTmp>>8)&0x0F)+1000*((CoPpmTmp>>12)&0x0F)+\
		10000*((CoPpmTmp>>16)&0x0F)+100000*((CoPpmTmp>>20)&0x0F);
		
		CoPpmTmp = 10* CoPpmTmp;
		for(i=5;i<16;i++){
			cTemp += CO_RX_BUF[i];
		}
		if(CO_RX_BUF[16] == cTemp ){
			CoPpm = filtStack(&CoPpmTmp,CoPpmBuf,&CoCnt,false);
			CoErrorCnt = 0;
			return true;
		}
	}
	if(CoErrorCnt++ > ErrorMax){
		CoPpm = 0;
		CoErrorCnt = ErrorMax;
	}

	return false;
}

int ReadNo2Ppm (void)
{
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	unsigned char cTemp =0;
	u32 No2PpmTmp = 0;
	for(i=0;i<11;i++)
	{
		rs485buf[i]=NO2_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,11);//发送11个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(NO2_RX_BUF,&revLen);//读取十八个数据
	if(revLen == 18)
	{
		//判断读回的数据是否是So2
		//so2 0x0A
		//no2 0x16
		if(0x0A == NO2_RX_BUF[14])
		{
			NO2_TX_BUF[3] = 0x03;
			SO2_TX_BUF[3] = 0x02;
			return false;
		}
			
		No2PpmTmp = (NO2_RX_BUF[10]<<16)|(NO2_RX_BUF[11]<<8)|NO2_RX_BUF[12];	 
		
		No2PpmTmp = (No2PpmTmp&0x0F)+10*((No2PpmTmp>>4)&0x0F)+\
		100*((No2PpmTmp>>8)&0x0F)+1000*((No2PpmTmp>>12)&0x0F)+\
		10000*((No2PpmTmp>>16)&0x0F)+100000*((No2PpmTmp>>20)&0x0F);
		
		for(i=5;i<16;i++){
			cTemp += NO2_RX_BUF[i];
		}
		if(NO2_RX_BUF[16] == cTemp ){
			No2Ppm = filtStack(&No2PpmTmp,No2PpmBuf,&No2Cnt,false);
			No2ErrorCnt = 0;
			return true;
		}
	}
	if(No2ErrorCnt++ > ErrorMax){
	No2Ppm = 0;
	No2ErrorCnt = ErrorMax;
	}
	return false;
}

int ReadSo2Ppm (void)
{
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	unsigned char cTemp =0;
	u32 So2PpmTmp = 0;
	for(i=0;i<11;i++)
	{
		rs485buf[i]=SO2_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,11);//发送11个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(SO2_RX_BUF,&revLen);//读取十八个数据
	if(revLen == 18)
	{
		//判断读回的数据是否是No2
		//so2 0x0A
		//no2 0x16
		if(0x16 == SO2_RX_BUF[14])
		{
			SO2_TX_BUF[3] = 0x02;
			NO2_TX_BUF[3] = 0x03;
			return false;
		}
		
		So2PpmTmp = (SO2_RX_BUF[10]<<16)|(SO2_RX_BUF[11]<<8)|SO2_RX_BUF[12];	 
		
		So2PpmTmp = (So2PpmTmp&0x0F)+10*((So2PpmTmp>>4)&0x0F)+\
		100*((So2PpmTmp>>8)&0x0F)+1000*((So2PpmTmp>>12)&0x0F)+\
		10000*((So2PpmTmp>>16)&0x0F)+100000*((So2PpmTmp>>20)&0x0F);
		
		for(i=5;i<16;i++){
			cTemp += SO2_RX_BUF[i];
		}
		if(SO2_RX_BUF[16] == cTemp ){
			So2Ppm = filtStack(&So2PpmTmp,So2PpmBuf,&So2Cnt,true);
			So2ErrorCnt = 0;
			return true;
		}
	}
	if(So2ErrorCnt++ > ErrorMax){
	So2Ppm = 0;
	So2ErrorCnt = ErrorMax;
	}
	return false;
}

int ReadO3Ppm (void)
{
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	unsigned char cTemp =0;
	u32 O3PpmTmp = 0;
	for(i=0;i<11;i++)
	{
		rs485buf[i]=O3_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,11);//发送11个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(O3_RX_BUF,&revLen);//读取十八个数据
	if(revLen == 18)
	{
		O3PpmTmp = (O3_RX_BUF[10]<<16)|(O3_RX_BUF[11]<<8)|O3_RX_BUF[12];	 
		
		O3PpmTmp = (O3PpmTmp&0x0F)+10*((O3PpmTmp>>4)&0x0F)+\
		100*((O3PpmTmp>>8)&0x0F)+1000*((O3PpmTmp>>12)&0x0F)+\
		10000*((O3PpmTmp>>16)&0x0F)+100000*((O3PpmTmp>>20)&0x0F);
		
		//O3PpmTmp = O3PpmTmp / 2.0;
		
		for(i=5;i<16;i++){
			cTemp += O3_RX_BUF[i];
		}
		if(O3_RX_BUF[16] == cTemp ){
			O3Ppm = filtStack(&O3PpmTmp,O3PpmBuf,&O3Cnt,false);
			O3ErrorCnt = 0;
			return true;
		}
	}
	if(O3ErrorCnt++ > ErrorMax){
	O3Ppm = 0;
	O3ErrorCnt = ErrorMax;
	}
	return false;
}

int ReadH2sPpm (void)
{
	unsigned char i=0;
	unsigned char revLen=0;
	unsigned char  rs485buf[64];
	unsigned char cTemp =0;
	u32 H2sPpmTmp = 0;
	for(i=0;i<11;i++)
	{
		rs485buf[i]=H2S_TX_BUF[i];//填充发送缓冲区
	}
	RS485_Send_Data(rs485buf,11);//发送11个字节 
	delay_ms(100);//要延迟否则接受数据中断
	RS485_Receive_Data(H2S_RX_BUF,&revLen);//读取十八个数据
	if(revLen == 18)
	{
		H2sPpmTmp = (O3_RX_BUF[10]<<16)|(O3_RX_BUF[11]<<8)|O3_RX_BUF[12];	 
		
		H2sPpmTmp = (H2sPpmTmp&0x0F)+10*((H2sPpmTmp>>4)&0x0F)+\
		100*((H2sPpmTmp>>8)&0x0F)+1000*((H2sPpmTmp>>12)&0x0F)+\
		10000*((H2sPpmTmp>>16)&0x0F)+100000*((H2sPpmTmp>>20)&0x0F);
				
		for(i=5;i<16;i++){
			cTemp += H2S_RX_BUF[i];
		}
		if(H2S_RX_BUF[16] == cTemp ){
			H2sPpm = filtStack(&H2sPpmTmp,H2sPpmBuf,&H2sCnt,false);
			H2sErrorCnt = 0;
			return true;
		}
	}	
	if(H2sErrorCnt++ > ErrorMax){
	H2sPpm = 0;
	H2sErrorCnt = ErrorMax;
	}
	return false;
}

int ReadAllToxicGas(void)
{
	ReadCoPpm ();
	ReadNo2Ppm ();
	ReadSo2Ppm ();
	ReadO3Ppm ();
	ReadH2sPpm ();
	ReadTvocPpm ();
	return true;
}


u32 filtStack(u32* source,u32 readFiltBuf[],u8* readCnt,u8 isRand)
{
		u32	PpmTmp = 0;
		int i =0;
	
		if(*readCnt < FLITE_NUM ){

		readFiltBuf[*readCnt] = *source;
		*readCnt = *readCnt + 1;
			
		for( i = 0 ; i< *readCnt; i++){
				PpmTmp += readFiltBuf[i] ; 
				}
				PpmTmp = PpmTmp / (*readCnt*1.0);
				if( isRand ){
						if(PpmTmp<0.5)
            {
								tmd = rand();
                PpmTmp = 5 + 2*((tmd-32767)/32767.0);//防止存储负值。
								//随机出值 范围 5 -- 3
            }
				}
				return PpmTmp; 					
		}else{

			for( i = FLITE_NUM -1 ; i>=0 ; i--){
			 readFiltBuf[i+1] = readFiltBuf[i];
			}
			
			readFiltBuf[0] = *source;

			for( i = 0 ; i<FLITE_NUM; i++){
				PpmTmp += readFiltBuf[i] ; 
			}

			PpmTmp = (float)PpmTmp / (FLITE_NUM*1.0);
			if( isRand ){
						if(PpmTmp<0.5)
            {
                PpmTmp = 5+ 2*((rand()-32767)/32767.0);//防止存储负值。
								//随机出值 范围 5 -- 3
            }
				}
			return PpmTmp; 
		}
}


					

