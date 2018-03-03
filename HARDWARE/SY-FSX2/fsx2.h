#ifndef __FSX2_H
#define __FSX2_H
#include "sys.h"
  
extern u16 windSpeed;
extern u16 windDir;

extern u16 LaserTsp;
extern u16 LaserPm10;
extern u16 LaserPm25;

extern u32 CoPpm ;
extern u32 No2Ppm ;
extern u32 So2Ppm ;
extern u32 O3Ppm ;
extern u32 TVOCPpm ;

int ReadWindSpeedDir(void);
int ReadUltraSound(void);
int ReadLaserTsp(void);
int ReadWindSpeed (void);
int ReadWindDir (void);
int ReadCoPpm (void);
int ReadAllToxicGas(void);
int ReadRsTemHum(void);
void SendLedScreamCmd(void);
#endif
