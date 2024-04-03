#include "main.h"
#include "adc.h"
#include "tim.h"
#include "user_parameters.h"
#include "../App/foc.h"
#include "../App/svpwm.h"
#include "../App/utils.h"
#include "../../Drivers/AS5047P/as5047p.h"
extern SVPWMTypeDef_t  SVPWM;
extern MATH_vec2 V_d_q, V_alpha_beta, I_d_q, I_alpha_beta, I_a_b;
extern Current_t current;
extern Voltage_t voltage;
extern Motor_t motor;
extern MATH_EMAVG_F math_emavg1;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        // Current_Get(&current);
        // motor.mag_angle = (float)(AS5047_read(ANGLECOM) * MATH_2PI / 16384.0f);
        // motor.elec_angle = Mag_To_Electrical(motor.mag_angle, NUM_OF_POLE_PAIRS);

        // Inv_Park_Run(&V_d_q, &V_alpha_beta, motor.elec_angle);

        // voltage.Valpha = V_alpha_beta.value[0];
        // voltage.Vbeta = V_alpha_beta.value[1];

        // I_a_b.value[0] = current.Ia;
        // I_a_b.value[1] = current.Ib;
        // Clarke_Run(&I_a_b, &I_alpha_beta);
        // current.Ialpha = I_alpha_beta.value[0];
        // current.Ibeta = I_alpha_beta.value[1];
        // Park_Run(&I_alpha_beta, &I_d_q, motor.elec_angle);

        // current.Id = I_d_q.value[0];
        // current.Iq = I_d_q.value[1];

        // SVPWM_Run(&SVPWM, voltage.Valpha, voltage.Vbeta, VBUS);
        // PWM_Set();
        //Drag_VF_Mode();
        // Drag_IF_Mode();
        // Voltage_Open_Loop();
        //Current_Closed_Loop();
        // Velocity_Closed_Loop();
        FOC_Run();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

    }
}
