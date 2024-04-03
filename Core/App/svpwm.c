#include "main.h"
#include "svpwm.h"

SectorEnum_t sector;
void SVPWM_Cal_Time(SVPWMTypeDef_t* pSVPWM)
{
    if (pSVPWM->T1 + pSVPWM->T2 > pSVPWM->Ts)
    {
        pSVPWM->T1 = pSVPWM->T1 * pSVPWM->Ts / (pSVPWM->T1 + pSVPWM->T2);
        pSVPWM->T2 = pSVPWM->T2 * pSVPWM->Ts / (pSVPWM->T1 + pSVPWM->T2);
    }
    //TODO:这里需要注意PWM模式和中心计数模式，这里选择模式1 和中心对齐向下计数是没问题的，忽略下方注释
    //PWM MODE  2 (set by STM32cubemx) //
    pSVPWM->Ta = (pSVPWM->Ts - pSVPWM->T1 - pSVPWM->T2) / 4;
    pSVPWM->Tb = pSVPWM->Ta + 0.5 * pSVPWM->T1;
    pSVPWM->Tc = pSVPWM->Tb + 0.5 * pSVPWM->T2;

    //PWM MODE 1 (or for simulation)
    // pSVPWM->Ta = (pSVPWM->Ts + pSVPWM->T1 + pSVPWM->T2) / 4;
    // pSVPWM->Tb = pSVPWM->Ta - 0.5 * pSVPWM->T1;
    // pSVPWM->Tc = pSVPWM->Tb - 0.5 * pSVPWM->T2;
}

void SVPWM_Run(SVPWMTypeDef_t* pSVPWM, float U_alpha, float U_beta, float U_dc)
{
    //clear N 
    pSVPWM->N = 0;
    pSVPWM->Ts = DUTYMAX;

    pSVPWM->Vref1 = U_beta;
    pSVPWM->Vref2 = MATH_SQRT3_2 * U_alpha - 0.5 * U_beta;
    pSVPWM->Vref3 = -MATH_SQRT3_2 * U_alpha - 0.5 * U_beta;

    pSVPWM->A = pSVPWM->Vref1 > 0;
    pSVPWM->B = pSVPWM->Vref2 > 0;
    pSVPWM->C = pSVPWM->Vref3 > 0;
    //A+2B+4C
    pSVPWM->N = pSVPWM->A + (pSVPWM->B << 1) + (pSVPWM->C << 2);
    // if (pSVPWM->Vref1 > 0) pSVPWM->N = pSVPWM->N + 1;
    // if (pSVPWM->Vref2 > 0) pSVPWM->N = pSVPWM->N + 2;
    // if (pSVPWM->Vref3 > 0) pSVPWM->N = pSVPWM->N + 4;

    pSVPWM->X = MATH_SQRT3 * pSVPWM->Ts / U_dc * U_beta;
    pSVPWM->Y = MATH_SQRT3 * pSVPWM->Ts / U_dc * (MATH_SQRT3_2 * U_alpha + 0.5 * U_beta);
    pSVPWM->Z = MATH_SQRT3 * pSVPWM->Ts / U_dc * (-MATH_SQRT3_2 * U_alpha + 0.5 * U_beta);

    switch (pSVPWM->N)
    {
    case Sector_1:
        pSVPWM->T1 = -pSVPWM->Z;
        pSVPWM->T2 = pSVPWM->X;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Ta;
        pSVPWM->Tcm2 = pSVPWM->Tb;
        pSVPWM->Tcm3 = pSVPWM->Tc;
        break;
    case Sector_2:
        pSVPWM->T1 = pSVPWM->Z;
        pSVPWM->T2 = pSVPWM->Y;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Tb;
        pSVPWM->Tcm2 = pSVPWM->Ta;
        pSVPWM->Tcm3 = pSVPWM->Tc;
        break;
    case Sector_3:
        pSVPWM->T1 = pSVPWM->X;
        pSVPWM->T2 = -pSVPWM->Y;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Tc;
        pSVPWM->Tcm2 = pSVPWM->Ta;
        pSVPWM->Tcm3 = pSVPWM->Tb;
        break;
    case Sector_4:
        pSVPWM->T1 = -pSVPWM->X;
        pSVPWM->T2 = pSVPWM->Z;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Tc;
        pSVPWM->Tcm2 = pSVPWM->Tb;
        pSVPWM->Tcm3 = pSVPWM->Ta;
        break;
    case Sector_5:
        pSVPWM->T1 = -pSVPWM->Y;
        pSVPWM->T2 = -pSVPWM->Z;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Tb;
        pSVPWM->Tcm2 = pSVPWM->Tc;
        pSVPWM->Tcm3 = pSVPWM->Ta;
        break;
    case Sector_6:
        pSVPWM->T1 = pSVPWM->Y;
        pSVPWM->T2 = -pSVPWM->X;
        SVPWM_Cal_Time(pSVPWM);
        pSVPWM->Tcm1 = pSVPWM->Ta;
        pSVPWM->Tcm2 = pSVPWM->Tc;
        pSVPWM->Tcm3 = pSVPWM->Tb;
        break;
    default:
        break;
    }

}

