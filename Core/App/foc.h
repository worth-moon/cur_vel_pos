#ifndef _FOC_H
#define _FOC_H

#define Compare_count(a, b) (((a) >= (b)) ? ((a) - (b)) : ((b) - (a)))
typedef enum
{
    LAB_1 = 1,
    LAB_2 = 2,
    LAB_3 = 3,
    LAB_4 = 4,
    LAB_5 = 5,
    LAB_6 = 6,
    LAB_7 = 7,
    LAB_8 = 8,
    LAB_9 = 9,
    LAB_10 = 10,
}FOC_e;

typedef struct
{
    int16_t Ia_raw, Ib_raw, Ic_raw, Ibus_raw;
    float Ia, Ib, Ic, Ibus;
    float Ialpha, Ibeta;
    float Id, Iq, Id_ref, Iq_ref;
    int16_t Ia_offset, Ib_offset, Ic_offset, Ibus_offset;
}Current_t;

typedef struct
{
    float Va, Vb, Vc, Vbus;
    float Valpha, Vbeta;
    float Vd, Vq;
}Voltage_t;
typedef  struct
{
    float mag_angle, prev_mag_angle, elec_angle;
    float self_angle, self_angle_acc;
    float vel, vel_filtered, omega_vel, vel_ref;
    FOC_e FOC_Lab;
    int cnt;
}Motor_t;

extern float debug_position_target;
extern float current_count;
extern float act;
extern float debug_vel_target;

void Current_Caloffset(Current_t* pcurr);
void Current_Get(Current_t* pcurr);
void Voltage_Caloffset(Voltage_t* pvol);
void Voltage_Get(Voltage_t* pvol);
void Voltage_Get(Voltage_t* pvol);
void Velocity_Get(Motor_t* pmotor, float delta_T);
void Drag_VF_Mode(void);
void Drag_IF_Mode(void);
void Voltage_Open_Loop(void);
void Current_Closed_Loop(void);
void Velocity_Closed_Loop(void);
void FOC_Init(void);
void FOC_Run(void);
void PWM_Set(void);
void PWM_Stop(void);


#endif // !
