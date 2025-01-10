#ifndef SIN_LUT_H
#define SIN_LUT_H

#include <stdint.h>

#define USE_HIGH_RES	1 //Seems to use an extra 30 clock cycles or so

void sin_cos_fast( uint16_t angle , float * sin, float * cos);
void getLabFast( uint16_t angle, float Ld, float Lq_Ld , float * La, float * Lb);

#endif