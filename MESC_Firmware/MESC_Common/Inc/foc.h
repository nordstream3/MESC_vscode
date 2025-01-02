#ifndef FOC_H
#define FOC_H

#include "MESCmotor.h"

typedef struct {
	float sin;
	float cos;
}MESCsin_cos_s;

typedef struct {
  float d;
  float q;
} MESCiq_s;

typedef struct {
  float a;
  float b;
  float g;
} MESCiab_s;

typedef struct {

  MESCsin_cos_s sincosangle;  // This variable carries the current sin and cosine of
                         	  // the angle being used for Park and Clark transforms,
                              // so they only need computing once per pwm cycle
  MESCiab_s Vab;							//Float vector containing the Alpha beta frame voltage
  MESCiab_s Iab;							// Float vector containing the Clark transformed current in Amps
  MESCiq_s Idq;      						// Float vector containing the Park
  	  	  	  	  	  	  	  	  	  	  	// transformed current in amps
  MESCiq_s Vdq;
  MESCiq_s Idq_smoothed;
  MESCiq_s Idq_int_err;

  float pwm_frequency;
  MESCiq_s Idq_req;							//The input to the PI controller. Load this with the values you want.
  uint16_t FOCAngle;
  //float hfi_voltage;
  //float Current_bandwidth;
  float PLL_error;
  float PLL_int;
  float PLL_kp;
  float PLL_ki;
  uint32_t PLL_angle;
} Base_foc_s;

#endif
