#ifndef ENCODER_H
#define ENCODER_H

#include "stm32fxxx_hal.h"

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;

void tle5012(MESC_motor_typedef *_motor);
void getIncEncAngle(MESC_motor_typedef *_motor);

#endif
