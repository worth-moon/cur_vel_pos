#ifndef _SVPWM_H_
#define _SVPWM_H_
#include "math.h"
#include "utils.h"
#define DUTYMAX (0.95f) 

typedef struct
{
    float Vref1, Vref2, Vref3;
    int A, B, C, N;
    float X, Y, Z;
    float Ts, T1, T2;
    float Ta, Tb, Tc;
    float Tcm1, Tcm2, Tcm3;
}SVPWMTypeDef_t;


typedef enum
{
    Sector_1 = 3,
    Sector_2 = 1,
    Sector_3 = 5,
    Sector_4 = 4,
    Sector_5 = 6,
    Sector_6 = 2,
}SectorEnum_t;



void SVPWM_Cal_Time(SVPWMTypeDef_t* pSVPWM);
void SVPWM_Run(SVPWMTypeDef_t* pSVPWM, float U_alpha, float U_beta, float U_dc);



#endif // !


