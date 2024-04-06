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
				HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,1);
        FOC_Run();
				//HAL_GPIO_TogglePin(CH1_GPIO_Port,CH1_Pin);
        HAL_GPIO_WritePin(CH1_GPIO_Port,CH1_Pin,0);

    }
}
