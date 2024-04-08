#ifndef _MT6816_H
#define _MT6816_H

#include <stdbool.h>		
#include <spi.h>		
#include <main.h>		

//一阶滤波 sample: 现在值, value:过去值, filter_constant:滤波系数
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))
uint16_t enc_mt6816_routine(void);




#endif 