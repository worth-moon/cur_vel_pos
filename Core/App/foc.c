#include "main.h"
#include "tim.h"
#include "adc.h"
#include "../../AS5047P/as5047p.h"
#include "foc.h"
#include "svpwm.h"
#include "utils.h"
#include "pid.h"
#include "user_parameters.h"
#include "mt6816.h"

SVPWMTypeDef_t  SVPWM;
MATH_vec2 V_d_q, V_alpha_beta, I_d_q, I_alpha_beta, I_a_b;
Current_t current;
Voltage_t voltage;
Motor_t motor;
MATH_EMAVG_F math_emavg1;
Pid_Controller_t GI_D;
Pid_Controller_t GI_Q;
Pid_Controller_t GVEL;
Pid_Controller_t GPOS;
float debug_p_d = 0.140, debug_i_d = 706.3372224;
float debug_p_q = 0.160, debug_i_q = 706.3372224;
float debug_target_d = 0.0, debug_target_q = 1.0;
float debug_position_output;

float debug_position_target = 3.14;
float debug_position_p = 5, debug_position_i = 0.0;
float current_count;
float act,last_act;
float debug_vel_target = 1000;
float debug_vel_filter = 1.0f;//当前占比
void Current_Caloffset(Current_t* pcurr)
{
    int16_t a, b, c, bus;
    for (int i = 0; i < 16; ++i)
    {
        HAL_Delay(1);
        pcurr->Ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        pcurr->Ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        pcurr->Ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
        pcurr->Ibus_offset = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
        a += pcurr->Ia_raw;
        b += pcurr->Ib_raw;
        c += pcurr->Ic_raw;
        bus += pcurr->Ibus_offset;
    }
    //右移四位，相当于 ÷16
    pcurr->Ia_offset = a >> 4;
    pcurr->Ib_offset = b >> 4;
    pcurr->Ic_offset = c >> 4;
    pcurr->Ibus_offset = bus >> 4;
}
void Current_Get(Current_t* pcurr)
{
    pcurr->Ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    pcurr->Ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    pcurr->Ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    pcurr->Ibus_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
    // pcurr->Ia = (float)(pcurr->Ia_raw - pcurr->Ia_offset);
    pcurr->Ia = (float)(pcurr->Ia_raw - pcurr->Ia_offset) * ADC_REF / ADC_12BIT / (CURRENT_RS * OPAMP_AU);
    pcurr->Ib = (float)(pcurr->Ib_raw - pcurr->Ib_offset) * ADC_REF / ADC_12BIT / (CURRENT_RS * OPAMP_AU);
    pcurr->Ic = (float)(pcurr->Ic_raw - pcurr->Ic_offset) * ADC_REF / ADC_12BIT / (CURRENT_RS * OPAMP_AU);
    pcurr->Ibus = (float)(pcurr->Ibus_raw - pcurr->Ibus_offset) * ADC_REF / ADC_12BIT / (CURRENT_RS * OPAMP_AU);
}
void Voltage_Caloffset(Voltage_t* pvol)
{
    int16_t a, b, c, bus;
    for (int i = 0; i < 16; ++i)
    {
        HAL_Delay(1);

    }
    //右移四位，相当于 ÷16
}
void Voltage_Get(Voltage_t* pvol)
{

}
void Velocity_Get(Motor_t* pmotor, float delta_T)
{
    // float diff;
    // diff = pmotor->mag_angle - pmotor->prev_mag_angle;
    // //pmotor->dir = (diff > 0) ? CW : CCW;
    // pmotor->omega_vel = (pmotor->mag_angle - pmotor->prev_mag_angle) / delta_T;

    // if (diff > MATH_PI) pmotor->omega_vel = (pmotor->mag_angle - pmotor->prev_mag_angle - MATH_2PI) / delta_T;
    // if (diff < -MATH_PI) pmotor->omega_vel = (pmotor->mag_angle - pmotor->prev_mag_angle + MATH_2PI) / delta_T;

    // pmotor->vel = pmotor->omega_vel * 30.0f / MATH_PI;

    // pmotor->prev_mag_angle = pmotor->mag_angle;

}
/**
 * @brief FOC hardware init
 *
 */
void FOC_Init(void)
{
    //
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    //
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 3350);

    HAL_ADCEx_InjectedStart_IT(&hadc1);

    HAL_Delay(100);
    Current_Caloffset(&current);
    //
    motor.self_angle_acc = SELF_ANGLE_ACC;

    MATH_EMAVG_F_Init(&math_emavg1);

    motor.FOC_Lab = 8;

    Pid_Init(&GI_D, debug_p_d, debug_i_d, 0, GI_D_KIS, 1.0f / GI_D_FREQUENCY, GI_D_RANGE);
    Pid_Init(&GI_Q, debug_p_q, debug_i_q, 0, GI_Q_KIS, 1.0f / GI_D_FREQUENCY, GI_Q_RANGE);
    Pid_Init(&GPOS, debug_position_p, debug_position_i, 0, GI_Q_KIS, 1.0f / 5000.0f, GI_Q_RANGE);
    Pid_Init(&GVEL, 0.02f, GVEL_KI, GVEL_KD, GVEL_KIS, 1.0f / 8333.3333f, 3.0f);
}


void Drag_VF_Mode(void)
{
    voltage.Vd = 0.0f;
    voltage.Vq = 1.0f;

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    motor.self_angle += motor.self_angle_acc;
    if (motor.self_angle > MATH_2PI) motor.self_angle = 0.0f;
    else if (motor.self_angle < 0) motor.self_angle = MATH_2PI;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.self_angle);

    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);

    PWM_Set();
}

void Drag_IF_Mode(void)
{
    Current_Get(&current);

    motor.self_angle += motor.self_angle_acc;
    if (motor.self_angle > MATH_2PI) motor.self_angle = 0.0f;
    else if (motor.self_angle < 0) motor.self_angle = MATH_2PI;

    I_a_b.value[0] = current.Ia;
    I_a_b.value[1] = current.Ib;

    Clarke_Run(&I_a_b, &I_alpha_beta);

    current.Ialpha = I_alpha_beta.value[0];
    current.Ibeta = I_alpha_beta.value[1];

    Park_Run(&I_alpha_beta, &I_d_q, motor.self_angle);

    current.Id = I_d_q.value[0];
    current.Iq = I_d_q.value[1];

    current.Id_ref = CURRENT_ID_REF;
    current.Iq_ref = CURRENT_IQ_REF;

    //voltage.Vd = 0.0f;
    //voltage.Vq = 1.5f;

    voltage.Vd = Pid_Cal(&GI_D, current.Id_ref, current.Id);
    voltage.Vq = Pid_Cal(&GI_Q, current.Iq_ref, current.Iq);

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.self_angle);


    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
    PWM_Set();
}

void Voltage_Open_Loop(void)
{
    voltage.Vd = 0.0f;
    voltage.Vq = 1.0f;

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;


    motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
    motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);

    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);

    PWM_Set();
}

//TODO:闭环需要轻微捏住电机才能够较好整定，感觉像是编码器的问题，精度不够高，转速过快
void Current_Closed_Loop(void)
{
    Current_Get(&current);
    motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
    motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

    I_a_b.value[0] = current.Ia;
    I_a_b.value[1] = current.Ib;

    Clarke_Run(&I_a_b, &I_alpha_beta);

    current.Ialpha = I_alpha_beta.value[0];
    current.Ibeta = I_alpha_beta.value[1];

    Park_Run(&I_alpha_beta, &I_d_q, motor.elec_angle);

    current.Id = I_d_q.value[0];
    current.Iq = I_d_q.value[1];

    current.Id_ref = debug_target_d;
    current.Iq_ref = debug_target_q;

    voltage.Vd = Pid_Cal(&GI_D, current.Id_ref, current.Id);
    voltage.Vq = Pid_Cal(&GI_Q, current.Iq_ref, current.Iq);

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);


    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
    PWM_Set();
}

void velocity(void)
{
    static uint8_t vel_cnt = 0;
    vel_cnt++;
    motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
    motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);
    static float last_angle = 10086;
    if(last_angle - motor.mag_angle>3.14f&&last_angle!=10086)
        current_count++;
    if (last_angle - motor.mag_angle < -3.14f && last_angle != 10086)
        current_count--;
 
    if (vel_cnt >= 5)
    {
        vel_cnt = 0;

        last_act = act;
        act = current_count * 6.28f + motor.mag_angle;
        static uint32_t time,last_time;
        last_time = time;
        time = HAL_GetTick();
        motor.vel = (act - last_act)/(time - last_time);
        //Velocity_Get(&motor, 1.0f / GVEL_FREQUENCY);

        math_emavg1.In = motor.vel;
        math_emavg1.Multiplier = debug_vel_filter;
        MATH_EMAVG_F_Run(&math_emavg1);
        motor.vel_filtered = math_emavg1.Out;

        motor.vel_ref = debug_vel_target;
    }

    voltage.Vd = 0;
    voltage.Vq = Pid_Cal(&GVEL, motor.vel_ref, motor.vel_filtered);

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);


    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
    PWM_Set();

}
void Velocity_Closed_Loop(void)
{
    static uint8_t vel_cnt = 0;
    Current_Get(&current);
    motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
    motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

    I_a_b.value[0] = current.Ia;
    I_a_b.value[1] = current.Ib;

    Clarke_Run(&I_a_b, &I_alpha_beta);

    current.Ialpha = I_alpha_beta.value[0];
    current.Ibeta = I_alpha_beta.value[1];

    Park_Run(&I_alpha_beta, &I_d_q, motor.elec_angle);

    current.Id = I_d_q.value[0];
    current.Iq = I_d_q.value[1];

     motor.cnt = vel_cnt;
     //Frequency 5000Hz
    vel_cnt++;
    if (vel_cnt >= 3)
    {

        vel_cnt = 0;
        Velocity_Get(&motor, 1.0f / GVEL_FREQUENCY);

        math_emavg1.In = motor.vel;
        math_emavg1.Multiplier = MATH_LPF_COEFF;
        MATH_EMAVG_F_Run(&math_emavg1);
        motor.vel_filtered = math_emavg1.Out;

        motor.vel_ref = debug_vel_target;
        current.Id_ref = 0.0f;
        current.Iq_ref = Pid_Cal(&GVEL, motor.vel_ref, motor.vel_filtered);
    }

    voltage.Vd = Pid_Cal(&GI_D, current.Id_ref, current.Id);
    voltage.Vq = Pid_Cal(&GI_Q, current.Iq_ref, current.Iq);

    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);


    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
    PWM_Set();

}
//单位置环已调好
void position_max(void)
{
    motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
    motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

    static float last_angle = 10086;
    if(last_angle - motor.mag_angle>4.0f&&last_angle!=10086)
        current_count++;
    if (last_angle - motor.mag_angle < -4.0f && last_angle != 10086)
        current_count--;

     act = current_count * 6.28f + motor.mag_angle;
    last_angle = motor.mag_angle;
    voltage.Vd = 0.0f;
    voltage.Vq = Pid_Cal(&GPOS, debug_position_target, act);
    V_d_q.value[0] = voltage.Vd;
    V_d_q.value[1] = voltage.Vq;

    Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);

    voltage.Valpha = V_alpha_beta.value[0];
    voltage.Vbeta = V_alpha_beta.value[1];

    SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, 24.4f);

    PWM_Set();
}


//电流环频率 25k    0.00004s=0.04ms=40us
//速度环频率 5k     0.0002s=0.2ms=200us
void current_position(void)
{
    static uint8_t vel_cnt = 0;
    vel_cnt++;
    if(vel_cnt == 3)
    {
        vel_cnt = 0;

        Current_Get(&current);
        motor.mag_angle = (float)(MT6816_Get_AngleData() * MATH_2PI / 16384.0f);
        motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

        I_a_b.value[0] = current.Ia;
        I_a_b.value[1] = current.Ib;

        Clarke_Run(&I_a_b, &I_alpha_beta);

        current.Ialpha = I_alpha_beta.value[0];
        current.Ibeta = I_alpha_beta.value[1];

        Park_Run(&I_alpha_beta, &I_d_q, motor.elec_angle);

        current.Id = I_d_q.value[0];
        current.Iq = I_d_q.value[1];

            static float last_angle = 10086;
            if(last_angle - motor.mag_angle > 3.0f&&last_angle!=10086)
                current_count++;
            if (last_angle - motor.mag_angle < -3.0f && last_angle != 10086)
                current_count--;

            act = current_count * 6.28f + motor.mag_angle;
            last_angle = motor.mag_angle;

        current.Id_ref = debug_target_d;
        current.Iq_ref = Pid_Cal(&GPOS, debug_position_target, act);

        voltage.Vd = Pid_Cal(&GI_D, current.Id_ref, current.Id);
        voltage.Vq = Pid_Cal(&GI_Q, current.Iq_ref, current.Iq);

        V_d_q.value[0] = voltage.Vd;
        V_d_q.value[1] = voltage.Vq;

        Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);


        voltage.Valpha = V_alpha_beta.value[0];
        voltage.Vbeta = V_alpha_beta.value[1];

        SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
        PWM_Set();

    }
}

void PWM_Set(void)
{
    TIM1->CCR1 = (uint16_t)roundf(SVPWM.Tcm1 * TIM1->ARR);
    TIM1->CCR2 = (uint16_t)roundf(SVPWM.Tcm2 * TIM1->ARR);
    TIM1->CCR3 = (uint16_t)roundf(SVPWM.Tcm3 * TIM1->ARR);
}

void PWM_Stop(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}
//TODO:挖一个大坑，接入无感部分
void FOC_Run(void)
{
    switch (motor.FOC_Lab)
    {
    case LAB_1:
        Drag_VF_Mode();
        break;
    case LAB_2:
        Drag_IF_Mode();
        break;
    case LAB_3:
        Voltage_Open_Loop();
        break;
    case LAB_4:
        Current_Closed_Loop();
        break;
    case LAB_5:
        position_max();
        break;
    case LAB_6:
        current_position();
        break;
    case LAB_7:
        Velocity_Closed_Loop();
        break;
    case LAB_8:
        velocity();
        break;
    case LAB_9:
        
        break;
    case LAB_10:
        
        break;
    default:
        break;
    }
}