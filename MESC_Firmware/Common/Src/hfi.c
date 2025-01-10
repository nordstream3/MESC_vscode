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


static void ToggleHFI(MESC_motor_typedef *_motor);


//volatile float min1=1000.0f;
//volatile float max1=0.0f;

void RunHFI(MESC_motor_typedef *_motor) {
  if ((hfi.inject)&&(_motor->MotorState != MOTOR_STATE_TRACKING)) {
	int Idqreq_dir=0;
	if (hfi.inject_high_low_now == 0){//First we create the toggle
		hfi.inject_high_low_now = 1;
		  Idq[0].d = foc.Idq.d;
		  Idq[0].q = foc.Idq.q;
	}else{
		hfi.inject_high_low_now = 0;
		  Idq[1].d = foc.Idq.d;
		  Idq[1].q = foc.Idq.q;
	  }
	hfi.didq.d = (Idq[0].d - Idq[1].d); //Calculate the changing current levels
	hfi.didq.q = (Idq[0].q - Idq[1].q);

	switch(_motor->HFIType){
		case HFI_TYPE_NONE:
		__NOP();
		break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_45:
			if(hfi.inject_high_low_now ==1){
				hfi.Vd_injectionV = +_motor->meas.hfi_voltage;
				if(foc.Idq_req.q>0.0f){
					Idqreq_dir = 1;
					hfi.Vq_injectionV = +_motor->meas.hfi_voltage;
				}else{
					hfi.Vq_injectionV = -_motor->meas.hfi_voltage;
					Idqreq_dir = -1;
				}
			}else{
				hfi.Vd_injectionV = -_motor->meas.hfi_voltage;
				if(foc.Idq_req.q>0.0f){
					hfi.Vq_injectionV = -_motor->meas.hfi_voltage;
				}else{
					hfi.Vq_injectionV = +_motor->meas.hfi_voltage;
				}
			}
			//Run the PLL
			magnitude45 = sqrtf(hfi.didq.d*hfi.didq.d+hfi.didq.q*hfi.didq.q);

			if(hfi.was_last_tracking==0){

				float error;
				//Estimate the angle error, the gain to be determined in the HFI detection and setup based on the HFI current and the max iteration allowable
				error = hfi.HFI_Gain*(magnitude45-hfi.HFI45_mod_didq);
				if(error>500.0f){error = 500.0f;}
				if(error<-500.0f){error = -500.0f;}
				hfi.HFI_int_err = hfi.HFI_int_err +0.05f*error;
				if(hfi.HFI_int_err>1000.0f){hfi.HFI_int_err = 1000.0f;}
				if(hfi.HFI_int_err<-1000.0f){hfi.HFI_int_err = -1000.0f;}
				foc.FOCAngle = foc.FOCAngle + (int)(error + hfi.HFI_int_err)*Idqreq_dir;

			}else{
				foc.FOCAngle += hfi.HFI_test_increment;
				hfi.HFI_accu += magnitude45;
				hfi.HFI_count += 1;
			}
			#if 0 //Sometimes for investigation we want to just lock the angle, this is an easy bodge
							foc.FOCAngle = 62000;
			#endif
				break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_D:
			if(hfi.inject_high_low_now ==1){
			  hfi.Vd_injectionV = +_motor->meas.hfi_voltage;
			}else{
			  hfi.Vd_injectionV = -_motor->meas.hfi_voltage;
			}
			if(hfi.didq.q>1.0f){hfi.didq.q = 1.0f;}
			if(hfi.didq.q<-1.0f){hfi.didq.q = -1.0f;}
			intdidq.q = (intdidq.q + 0.1f*hfi.didq.q);
			if(intdidq.q>10){intdidq.q=10;}
			if(intdidq.q<-10){intdidq.q=-10;}
			foc.FOCAngle += (int)(250.0f*hfi.IIR[1] + 10.50f*intdidq.q)*hfi.d_polarity;
		break;
		case HFI_TYPE_SPECIAL:
			__NOP();
			if(hfi.inject_high_low_now ==1){
			  hfi.Vd_injectionV = hfi.special_injectionVd;
			  hfi.Vq_injectionV = hfi.special_injectionVq;
			}else{
				hfi.Vd_injectionV = -hfi.special_injectionVd;
				hfi.Vq_injectionV = -hfi.special_injectionVq;			}
		break;
	}
  }else {
	  hfi.Vd_injectionV = 0.0f;
	  hfi.Vq_injectionV = 0.0f;
  }
}

void ToggleHFI(MESC_motor_typedef *_motor) {
	if(((foc.Vdq.q-foc.Idq_smoothed.q*_motor->m.R) > hfi.HFI_toggle_voltage)||((foc.Vdq.q-foc.Idq_smoothed.q*_motor->m.R) < -hfi.HFI_toggle_voltage)||(_motor->MotorSensorMode==MOTOR_SENSOR_MODE_HALL)){
		hfi.inject = 0;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
	} else if(((foc.Vdq.q-foc.Idq_smoothed.q*_motor->m.R) < (hfi.HFI_toggle_voltage-1.0f))&&((foc.Vdq.q-foc.Idq_smoothed.q*_motor->m.R) > -(hfi.HFI_toggle_voltage-1.0f)) &&(_motor->HFIType !=HFI_TYPE_NONE)){
		hfi.HFI_int_err = foc.PLL_int;
		hfi.inject = 1;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH*0.1f;
	}
}

void SlowHFI(MESC_motor_typedef *_motor) {
	/////////////Set and reset the HFI////////////////////////
		switch(_motor->HFIType){
			case HFI_TYPE_45:
			ToggleHFI(_motor);
				if(hfi.inject==1){
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
				ToggleHFI(_motor);
				if(hfi.inject==1){
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
				hfi.inject = 0;
//				foc.Current_bandwidth = CURRENT_BANDWIDTH;
			break;
			case HFI_TYPE_SPECIAL:
				ToggleHFI(_motor);
			break;
		}
}
