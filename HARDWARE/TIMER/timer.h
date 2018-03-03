#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
 
// 定时器配置
//T3 编码器A
//T4 编码器B
//T3 1s 定时器心跳包 计数用
//T5 0.3ms 定时器 FPGA 帧包计时
extern volatile  unsigned char One_second ;	 

void TIM2_Int_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc); 
//void Tim3_PWMIC_Init(void);
#endif
