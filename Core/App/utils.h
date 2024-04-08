#ifndef _UTILS_H
#define _UTILS_H
#include "arm_math.h"
#define MATH_SQRT3     (1.7320508075688772935274463415059f)
#define MATH_SQRT3_2   (0.86602540378443864676372317075294f)
#define MATH_PI (3.14159265f)
#define MATH_2PI (6.2831853071f)
#define MATH_CIRCLE (360.0f)
#define MATH_LPF_COEFF  (0.3f)
typedef struct
{
    float  value[2];
} MATH_vec2;

typedef struct
{
    float  value[3];
} MATH_vec3;


typedef struct {
    float In;
    float Out;
    float Multiplier;
} MATH_EMAVG_F;

static inline void Clarke_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec);
static inline void Park_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec, float theta);
static inline void Inv_Park_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec, float theta);
static inline float Mag_To_Electrical(float angle, float pole_pairs);
static inline void MATH_EMAVG_F_Init(MATH_EMAVG_F* v);
static inline void MATH_EMAVG_F_Run(MATH_EMAVG_F* v);

//TODO:后续三角函数计算需要优�?
static inline void Clarke_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec)
{
    pOutVec->value[0] = pInVec->value[0];
    pOutVec->value[1] = (pInVec->value[0] + 2.0f * pInVec->value[1]) / MATH_SQRT3;
}


static inline void Park_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec, float theta)
{
    pOutVec->value[0] = arm_cos_f32(theta) * pInVec->value[0] + arm_sin_f32(theta) * pInVec->value[1];
    pOutVec->value[1] = -arm_sin_f32(theta) * pInVec->value[0] + arm_cos_f32(theta) * pInVec->value[1];
}

static inline void Inv_Park_Run(MATH_vec2* pInVec, MATH_vec2* pOutVec, float theta)
{
    pOutVec->value[0] = arm_cos_f32(theta) * pInVec->value[0] - arm_sin_f32(theta) * pInVec->value[1];
    pOutVec->value[1] = arm_sin_f32(theta) * pInVec->value[0] + arm_cos_f32(theta) * pInVec->value[1];
}



static inline float Mag_To_Electrical(float angle, float pole_pairs)
{
    float a = fmodf(angle * pole_pairs, MATH_2PI);
    return a >= 0 ? a : (a + MATH_2PI);
}

static inline void MATH_EMAVG_F_Init(MATH_EMAVG_F* v)
{
    v->In = 0;
    v->Out = 0;
    v->Multiplier = 0;
}

static inline void MATH_EMAVG_F_Run(MATH_EMAVG_F* v)
{
    v->Out = v->In  * v->Multiplier + v->Out*(1 - v->Multiplier);
}
#endif // !
