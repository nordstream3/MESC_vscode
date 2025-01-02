
#ifndef HFI_H
#define HFI_H

#include "foc.h"

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;

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
}HFI_type_e;

typedef struct {
  //HFI
  uint16_t inject;
  uint16_t inject_high_low_now;
  float Vd_injectionV;
  float Vq_injectionV;
  float special_injectionVd;
  float special_injectionVq;
  float HFI_toggle_voltage;
  float HFI45_mod_didq;
  float HFI_Gain;
  float HFI_int_err;
  float HFI_accu;
  MESCiq_s didq;
  int32_t HFI_countdown;
  uint32_t HFI_count;
  uint32_t HFI_test_increment;
  int was_last_tracking;
  HFI_type_e HFIType;
  int d_polarity; //With this, we can swap the PLL polarity and therefore make it track Q instead of D. This is useful for detection
  float IIR[2];
  float Ldq_now_dboost[2];
  float Ldq_now[2];
} hfi_s;

/* Function prototypes -----------------------------------------------*/
void RunHFI(MESC_motor_typedef *_motor);
float detectHFI(MESC_motor_typedef *_motor);
void SlowHFI(MESC_motor_typedef *_motor);

#endif
