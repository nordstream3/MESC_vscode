#ifndef FOC_H
#define FOC_H

#include "stm32fxxx_hal.h"

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;

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


typedef struct  {
  int initing;  // Flag to say we are initialising

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
  float pwm_period;
  MESCiq_s Idq_req;							//The input to the PI controller. Load this with the values you want.
  uint16_t FOCAngle;
  float PLL_error;
  float PLL_int;
  float PLL_kp;
  float PLL_ki;
  uint32_t PLL_angle;

  float Current_bandwidth;
  float Vab_to_PWM;
  float Duty_scaler;
  float Voltage;
  float Vmag_max;
  float V_3Q_mag_max;
  float Vmag_max2;
  float Vd_max;
  float Vq_max;
  float Vdint_max;
  float Vqint_max;
  float PWMmid;
  uint32_t ADC_duty_threshold;

  float Id_pgain;  // Current controller gains
  float Id_igain;
  float Iq_pgain;
  float Iq_igain;

  uint16_t openloop_step;//The angle to increment by for openloop
  uint32_t encoder_duration;
  uint32_t encoder_pulse;
  uint32_t encoder_OK;
  uint16_t enc_angle;

  uint16_t enc_period_count;//For PWM encoder interpolation
  uint16_t enc_ratio;//For ABI encoder PPR to uint16_t conversion
  uint16_t last_enc_period;
  uint16_t last_enc_angle;
  int16_t enc_pwm_step;

  uint16_t enc_offset;
  float encsin;
  float enccos;
  uint16_t encoder_polarity_invert;
  int enc_obs_angle;
  uint16_t parkangle;
  float park_current;
  float park_current_now;

  float FLAdiff;
  float id_mtpa;
  float iq_mtpa;


  float inverterVoltage[3];
  MESCiq_s Idq_prereq2;
  MESCiq_s Idq_prereq; 						//Before we set the input to the current PI controller, we want to run a series of calcs (collect variables,
										  	  //calculate MTPA... that needs to be done without it putting jitter onto the PI input.
  float T_rollback;							//Scale the input parameters by this amount when thermal throttling
  MESCiq_s currentPower;					//Power being consumed by the motor; this does not include steady state losses and losses to switching
  float currentPowerab;
  float Ibus;
  float reqPower;
  float speed_req;
  float speed_kp;
  float speed_ki;
  float speed_error_int;

  //Observer parameters
  float Ia_last;
  float Ib_last;
  float La_last;
  float Lb_last;

  //Hall start
  uint16_t hall_initialised;
  int hall_start_now;

  //Encoder start
  int enc_start_now;

  // Field weakenning
  float FW_curr_max;
  float FW_threshold;
  float FW_multiplier;
  float FW_current;
  float FW_ehz_max;
  float FW_estep_max;

  float flux_a;
  float flux_b;
  float flux_observed;
  uint16_t state[4];  // current state, last state, angle change occurred
  uint16_t hall_update;
  uint32_t IRQentry;
  uint32_t IRQexit;

  float eHz;
  float mechRPM;

  uint32_t cycles_fastloop;
  uint32_t cycles_pwmloop;

  // Dead time compensation used in function writePWM() when macro DEADTIME_COMP is defined  
  uint16_t deadtime_comp;

} foc_s;

extern foc_s foc_vars;

// Anonymous union inside structure
//union { Base_foc_s FOC; MESCfoc_s FOC_; } Foc_u;


void MESCFOC(MESC_motor_typedef *_motor);  // Field and quadrature current control (PI?)
                 // Inverse Clark and Park transforms

void writePWM(MESC_motor_typedef *_motor);  // Offset the PWM to voltage centred (0Vduty is 50% PWM) or
                  // subtract lowest phase to always clamp one phase at 0V or
                  // SVPWM
                  // write CCR registers

void MESCTrack(MESC_motor_typedef *_motor);

void deadshort(MESC_motor_typedef *_motor);

void SlowStartup(MESC_motor_typedef *_motor);

void OLGenerateAngle(MESC_motor_typedef *_motor);  // For open loop FOC startup, just use this to generate
                         // an angle and velocity ramp, then keep the phase
                         // currents at the requested value without really
                         // thinking about things like synchronising, phase
                         // etc...

void RunMTPA(MESC_motor_typedef *_motor);

float fast_atan2(float y, float x);

#endif
