#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
 
// ��ʱ������
//T3 ������A
//T4 ������B
//T3 1s ��ʱ�������� ������
//T5 0.3ms ��ʱ�� FPGA ֡����ʱ
extern volatile  unsigned char One_second ;	 

void TIM2_Int_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc); 
//void Tim3_PWMIC_Init(void);
#endif
