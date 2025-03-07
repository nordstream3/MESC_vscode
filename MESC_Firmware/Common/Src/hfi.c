#include "hfi.h"
#include "sin_lut.h"
#include "error.h"
#include "motorinstance.h"
#include "interrupt_adc.h"
#include "conversions.h"
#include <math.h>
#include <stdlib.h>
#ifdef LOGGING
#include <stdio.h>
#endif

#define foc _motor->FOC
#define hfi _motor->hfi

// The hyperloop runs at PWM timer bottom, when the PWM is in V7 (all high)
// In this loop, we write the values of the PWM to be updated at the next update
// event (timer top) This is where we want to inject signals for measurement so
// that the next signal level takes affect right after the ADC reading In normal
// run mode, we want to increment the angle and write the next PWM values
static MESCiq_s Idq[2] = {{.d = 0.0f, .q = 0.0f}, {.d = 0.0f, .q = 0.0f}};
static volatile MESCiq_s dIdq = {.d = 0.0f, .q = 0.0f};
// static float IIR[2] = {0.0f, 0.0f};
static MESCiq_s intdidq;
static volatile float magnitude45;


void RunHFI(MESC_motor_typedef *_motor) {
  // Check if HFI injection is enabled and motor is not in tracking mode
  if ((hfi.do_injection) && (_motor->MotorState != MOTOR_STATE_TRACKING)) {

    // Toggle injection state
    if (hfi.injection_sign < 0) {
      hfi.injection_sign = 1;
      Idq[0].d = foc.Idq.d;
      Idq[0].q = foc.Idq.q;
    } else {
      hfi.injection_sign = -1;
      Idq[1].d = foc.Idq.d;
      Idq[1].q = foc.Idq.q;
    }

    // Compute differential current changes
    hfi.didq.d = (Idq[0].d - Idq[1].d);
    hfi.didq.q = (Idq[0].q - Idq[1].q);

    switch (_motor->HFIType) {
      case HFI_TYPE_NONE:
        __NOP();  // No operation if HFI is disabled
        break;

      case HFI_TYPE_45:
		// Acquire direction of requested input to PI controller
	    int Idqreq_dir = (foc.Idq_req.q > 0.0f) ? 1 : -1;

        // Apply 45-degree voltage injection
		hfi.Vd_injectionV = hfi.injection_sign * _motor->meas.hfi_voltage;
		hfi.Vq_injectionV = hfi.injection_sign * _motor->meas.hfi_voltage * Idqreq_dir;

        // Calculate magnitude of HFI current response
        magnitude45 = sqrtf(hfi.didq.d * hfi.didq.d + hfi.didq.q * hfi.didq.q);

		/* Are we not coming from a situation of standstill, or is this a TRUE startup ? */
        if (hfi.was_last_tracking == 0) {
			// Estimate the angle error, the gain to be determined in the HFI detection and setup
			// based on the HFI current and the max iteration allowable
			float error = hfi.HFI_Gain * (magnitude45 - hfi.HFI45_mod_didq);
			error = fminf(fmaxf(error, -500.0f), 500.0f);

			hfi.HFI_int_err = fminf(fmaxf(hfi.HFI_int_err + 0.05f * error, -1000.0f), 1000.0f);

			// Only apply angular update when actually injecting a voltage
			// IS THIS A BUG ?????
			if (hfi.injection_sign > 0) {
				foc.FOCAngle += (int)(error + hfi.HFI_int_err) * Idqreq_dir;
			}
        } else {
			foc.FOCAngle += hfi.HFI_test_increment;
			hfi.HFI_accu += magnitude45;
			hfi.HFI_count += 1;
        }
        #if 0 //Sometimes for investigation we want to just lock the angle, this is an easy bodge
			foc.FOCAngle = 62000;
        #endif
        break;

      case HFI_TYPE_D:
        // Apply D-axis injection
        hfi.Vd_injectionV = (hfi.injection_sign > 0) ? +_motor->meas.hfi_voltage : -_motor->meas.hfi_voltage;

        hfi.didq.q = fminf(fmaxf(hfi.didq.q, -1.0f), 1.0f);
        intdidq.q = fminf(fmaxf(intdidq.q + 0.1f * hfi.didq.q, -10.0f), 10.0f);
        foc.FOCAngle += (int)(250.0f * hfi.IIR[1] + 10.50f * intdidq.q) * hfi.d_polarity;
        break;

      case HFI_TYPE_SPECIAL:
        __NOP();
        hfi.Vd_injectionV = (hfi.injection_sign > 0) ? hfi.special_injectionVd : -hfi.special_injectionVd;
        hfi.Vq_injectionV = (hfi.injection_sign > 0) ? hfi.special_injectionVq : -hfi.special_injectionVq;
        break;
    }
  } else {
    hfi.Vd_injectionV = 0.0f;
    hfi.Vq_injectionV = 0.0f;
  }
}

/*
	The HFI toggling logic enables or disables High-Frequency Injection based on 
	whether the back-EMF is strong enough for rotor position estimation.
*/
void EvaluateHFI_on_off(MESC_motor_typedef *_motor) {

	float back_EMF_q = foc.Vdq.q - foc.Idq_smoothed.q*_motor->m.R;

	/* If back-EMF is large → The motor is likely spinning at a reasonable speed, so HFI is not needed. */
	if (		back_EMF_q  >  hfi.HFI_toggle_voltage
				|| back_EMF_q  < -hfi.HFI_toggle_voltage
				|| _motor->MotorSensorMode == MOTOR_SENSOR_MODE_HALL ) {
		
		hfi.do_injection = 0;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
	
	/* If back-EMF is small → The motor might be at low speed or stalled, so HFI is needed to estimate rotor position. */
	} else if ( back_EMF_q < (hfi.HFI_toggle_voltage-1.0f)
				&& back_EMF_q > -(hfi.HFI_toggle_voltage-1.0f)
				&& _motor->HFIType != HFI_TYPE_NONE ) {
		
		hfi.HFI_int_err = foc.PLL_int;
		hfi.do_injection = 1;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH*0.1f;
	}
}

void SlowHFI(MESC_motor_typedef *_motor) {
	/////////////Set and reset the HFI////////////////////////
		switch(_motor->HFIType){
			case HFI_TYPE_45:
				EvaluateHFI_on_off(_motor);
				if(hfi.do_injection==1){
					//static int no_q;
					if(hfi.was_last_tracking==1){
						if(hfi.HFI_countdown>1){
							hfi.HFI45_mod_didq = hfi.HFI_accu / hfi.HFI_count;
							hfi.HFI_Gain = 5000.0f/hfi.HFI45_mod_didq;
							hfi.was_last_tracking = 0;
						}else{
							hfi.HFI_test_increment = 65536 * SLOW_LOOP_FREQUENCY / foc.pwm_frequency;
							hfi.HFI_countdown++;
						}
					}else{
						hfi.HFI_countdown = 0;
						hfi.HFI_count = 0;
						hfi.HFI_accu = 0.0f;
					}
				}
			break;

			case HFI_TYPE_D:
				EvaluateHFI_on_off(_motor);
				if(hfi.do_injection==1){
					if(hfi.HFI_countdown==3){
						foc.Idq_req.d = HFI_TEST_CURRENT;
						foc.Idq_req.q=0.0f;//Override the inputs to set Q current to zero
					}else if(hfi.HFI_countdown==2){
						hfi.Ldq_now_dboost[0] = hfi.IIR[0]; //Find the effect of d-axis current
						foc.Idq_req.d = 1.0f;
						foc.Idq_req.q=0.0f;
					}else if(hfi.HFI_countdown == 1){
						foc.Idq_req.d = -HFI_TEST_CURRENT;
						foc.Idq_req.q=0.0f;
					}else if(hfi.HFI_countdown == 0){
						hfi.Ldq_now[0] = hfi.IIR[0];//hfi.Vd_injectionV;
						foc.Idq_req.d = 0.0f;
					if(hfi.Ldq_now[0]>hfi.Ldq_now_dboost[0]){foc.FOCAngle+=32768;}
					hfi.HFI_countdown = 200;
					}
					hfi.HFI_countdown--;
				}
			break;

			case HFI_TYPE_NONE:
				hfi.do_injection = 0;
//				foc.Current_bandwidth = CURRENT_BANDWIDTH;
			break;
			case HFI_TYPE_SPECIAL:
				EvaluateHFI_on_off(_motor);
			break;
		}
}
