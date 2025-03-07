#ifndef HFI_H
#define HFI_H

#include "foc.h"

// Forward declaration of MESC_motor_typedef
//typedef struct MESC_motor_typedef MESC_motor_typedef;

//HFI related
#ifndef HFI_VOLTAGE
#define HFI_VOLTAGE 4.0f
#endif

#ifndef HFI_TEST_CURRENT
#define HFI_TEST_CURRENT 10.0f
#endif

#ifndef HFI_THRESHOLD
#define HFI_THRESHOLD 3.0f
#endif

typedef enum {
  HFI_TYPE_NONE,
  HFI_TYPE_45,
  HFI_TYPE_D,
  HFI_TYPE_SPECIAL,
} HFI_type_e;

typedef struct { //HFI

  // Enable or disable HFI
  // 0 (off) / 1 (on) Controls whether HFI is running at all
  uint16_t do_injection;

  // Toggle between injection phases
  // 0 (low phase) / 1 (high phase) Controls the current state within an active HFI cycle
  int16_t injection_sign;
  
  float Vd_injectionV;
  float Vq_injectionV;
  float special_injectionVd;
  float special_injectionVq;
  float HFI_toggle_voltage;
  float HFI45_mod_didq;
  float HFI_Gain;
  float HFI_int_err;

  MESCiq_s didq;
  
  // INFO: slowLoop runs at SLOW_LOOP_FREQUENCY (hz)
  //       fastLoop runs at foc.pwm_frequency (hz)


  //volatile float magnitude45;

  

  // State Variable describing the transition from MOTOR_STATE_TRACKING to MOTOR_STATE_RUN. Managed entirely by slowLoop
  int was_last_tracking;

  // Time variable related to the transition from MOTOR_STATE_TRACKING to MOTOR_STATE_RUN. Managed entirely by slowLoop
  // HFI_TYPE_45: Only 1 slowLoop cycle
  int32_t HFI_countdown;

  // Accumulator and counter used for calculation of average "magnitude45".
  // Accumulation and counting Managed by fastLoop.
  float HFI_accu;
  uint32_t HFI_count;
  
  // The angle of a step/increment is: 2*pi * SLOW_LOOP_FREQUENCY / foc.pwm_frequency      (1.2 degrees)
  // This step is added to foc.FOCAngle (SLOW_LOOP_FREQUENCY / foc.pwm_frequency) times -> 2*pi
  // Disclaimer: Why isn't this a signed integer? The direction (sign) should then be aligned with the requested torque (Idq_req.q)
  uint32_t HFI_test_increment;

  HFI_type_e HFIType;
  int d_polarity; //With this, we can swap the PLL polarity and therefore make it track Q instead of D. This is useful for detection
  float IIR[2];
  float Ldq_now_dboost[2];
  float Ldq_now[2];
} hfi_s;

/* Function prototypes -----------------------------------------------*/
void RunHFI(MESC_motor_typedef *_motor);
void SlowHFI(MESC_motor_typedef *_motor);

#endif
