#include "response.h"
#include "delay.h"
#include "usart.h"	 
#include "string.h"
#include "fsx2.h"

unsigned char controlStep =0;

void CleanSent_Buffer()
{
 memset(USART_TX_BUF, 0, USART_REC_LEN );
}

void CleanRec_Buffer(unsigned char *dat)
{
	memset(dat, 0, USART_REC_LEN );
}

unsigned char CheackDat( unsigned char *dat)
{
//	unsigned char tmpc = 0,step = 0 ,i=0  ;
//	
//	for(i = 0;i<USART_REC_LEN;i++)
//	{
//		tmpc = dat[i];
//		switch(step)
//		{
//			case 0: 
//				if(START==tmpc)
//				{	
//					step=1;
//				}
//			break;	//帧开始
//			case 1:   //命令解析
//				switch(tmpc)
//				{
//					case FPGA_RESET: 
//						step = 2; 	
//					break;
//					case ENDFRAME:		   		break;					
//					case ADCCTRLS:				break;
//					case ADCCTRLM:	 			break;
//					case ADCCTRLC:		   		break;
//					case DACCTRLM:	 			break;
//					case DACCTRLC:		   		break;
//					case ADDACTRL:				break;
//					case TTLCTRLI:		   		break;
//					case TTLCTRLO:				break;
//					case SPICTRLI:	 			break;
//					case SPICTRLO:		   		break;
//					case ADBUFFCTRL: 
//						step = 4;	
//					break;
//					case ADBUFFREAD:	 		break;
//					case ADBUFFRLS:		   		break;
//					case USERBUFF:				break;
//					case USERWRITE:	 			break;
//					case USERREAD:		   		break;
//					case ADDASELFTEST:			break;
//					case COUNT_READ: 
//						step = 8; 	
//					break;
//					default :
//						step = 0;
//						i = USART_REC_LEN; 
//					break;
//				}
//			break;
//			 
//			case 2: 
//				switch (tmpc)
//				{
//					case ERR_OK: 
//						step = 3; 		
//					break;
//					default :
//						step = 0;
//						i = USART_REC_LEN; 
//					break;
//				}
//			break;
//			
//			case 3:
//				switch(tmpc)
//				{
//					//EF E9 00 EF
//					case STOP:
//						step = 0;
//						i = USART_REC_LEN;
//						CleanRec_Buffer(dat); 
//						controlStep =1;
//					break;
//					
//					default :
//						step = 0;
//						i = USART_REC_LEN; 
//						CleanRec_Buffer(dat); 
//					break;
//				}
//			break;	
//				
//			default :
//				step = 0;
//				i = USART_REC_LEN;	
//				CleanRec_Buffer(dat); 			
//			break;			
//		}							
//	}
// 
//	return 0;

	if( (USART_RX_STA == 10)&& (dat[0]==0xAA) && (dat[1]==0xC0))
	{
		LaserPm25 = dat[3]<<8 | dat[2] ;
		LaserPm10 = dat[5]<<8 | dat[4] ;
	}
	USART_RX_STA=0;
	CleanRec_Buffer(dat); 
		
	return 0;
}

//控制命令  
void Rest()
{    
	unsigned char len =0,t=0;
	CleanSent_Buffer();
	USART_TX_BUF[0] = START;
	USART_TX_BUF[1] = ACK;
	USART_TX_BUF[2] = 0;
	USART_TX_BUF[3] = STOP;
	len	= 4;				
	for(t=0;t<len;t++)
	{ 
		USART_SendData(USART1, USART_TX_BUF[t]);  
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		//等待发送结束
	}
}



//应答帧
void ACKpro(unsigned char  errorcode)
{     
	unsigned char len =0,t=0;
	CleanSent_Buffer();
	USART_TX_BUF[0] = START;
	USART_TX_BUF[1] = ACK;
	USART_TX_BUF[2] = errorcode;
	USART_TX_BUF[3] = STOP;
	len	= 4;				
	for(t=0;t<len;t++)
	{ 
		USART_SendData(USART1, USART_TX_BUF[t]);  
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		//等待发送结束
	}
}


//结束帧
void ENDpro()
{    
	unsigned char len =0,t=0;
	CleanSent_Buffer();
	USART_TX_BUF[0] = START;
	USART_TX_BUF[1] = ACK;
	USART_TX_BUF[2] = ENDFRAME;
	USART_TX_BUF[3] = STOP;
	len	= 4;				
	for(t=0;t<len;t++)
	{ 
		USART_SendData(USART1, USART_TX_BUF[t]);  
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		//等待发送结束
	}
}


	
