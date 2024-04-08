#ifndef _MT6816_H
#define _MT6816_H

#include <stdbool.h>		
#include <spi.h>		
#include <main.h>		

//һ���˲� sample: ����ֵ, value:��ȥֵ, filter_constant:�˲�ϵ��
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))
uint16_t enc_mt6816_routine(void);




#endif 