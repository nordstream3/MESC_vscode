#include "hfi.h"

#include "MESCfoc.h"

#include "MESCsin_lut.h"
#include "MESCerror.h"
#include "conversions.h"

#include <math.h>
#include <stdlib.h>
#ifdef LOGGING
#include <stdio.h>
#endif

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
  if ((_motor->hfi.inject)&&(_motor->MotorState != MOTOR_STATE_TRACKING)) {
	int Idqreq_dir=0;
	if (_motor->hfi.inject_high_low_now == 0){//First we create the toggle
		_motor->hfi.inject_high_low_now = 1;
		  Idq[0].d = _motor->FOC.base.Idq.d;
		  Idq[0].q = _motor->FOC.base.Idq.q;
	}else{
		_motor->hfi.inject_high_low_now = 0;
		  Idq[1].d = _motor->FOC.base.Idq.d;
		  Idq[1].q = _motor->FOC.base.Idq.q;
	  }
	_motor->hfi.didq.d = (Idq[0].d - Idq[1].d); //Calculate the changing current levels
	_motor->hfi.didq.q = (Idq[0].q - Idq[1].q);

	switch(_motor->HFIType){
		case HFI_TYPE_NONE:
		__NOP();
		break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_45:
			if(_motor->hfi.inject_high_low_now ==1){
				_motor->hfi.Vd_injectionV = +_motor->meas.hfi_voltage;
				if(_motor->FOC.base.Idq_req.q>0.0f){
					Idqreq_dir = 1;
					_motor->hfi.Vq_injectionV = +_motor->meas.hfi_voltage;
				}else{
					_motor->hfi.Vq_injectionV = -_motor->meas.hfi_voltage;
					Idqreq_dir = -1;
				}
			}else{
				_motor->hfi.Vd_injectionV = -_motor->meas.hfi_voltage;
				if(_motor->FOC.base.Idq_req.q>0.0f){
					_motor->hfi.Vq_injectionV = -_motor->meas.hfi_voltage;
				}else{
					_motor->hfi.Vq_injectionV = +_motor->meas.hfi_voltage;
				}
			}
			//Run the PLL
			magnitude45 = sqrtf(_motor->hfi.didq.d*_motor->hfi.didq.d+_motor->hfi.didq.q*_motor->hfi.didq.q);

			if(_motor->hfi.was_last_tracking==0){

				float error;
				//Estimate the angle error, the gain to be determined in the HFI detection and setup based on the HFI current and the max iteration allowable
				error = _motor->hfi.HFI_Gain*(magnitude45-_motor->hfi.HFI45_mod_didq);
				if(error>500.0f){error = 500.0f;}
				if(error<-500.0f){error = -500.0f;}
				_motor->hfi.HFI_int_err = _motor->hfi.HFI_int_err +0.05f*error;
				if(_motor->hfi.HFI_int_err>1000.0f){_motor->hfi.HFI_int_err = 1000.0f;}
				if(_motor->hfi.HFI_int_err<-1000.0f){_motor->hfi.HFI_int_err = -1000.0f;}
				_motor->FOC.base.FOCAngle = _motor->FOC.base.FOCAngle + (int)(error + _motor->hfi.HFI_int_err)*Idqreq_dir;

			}else{
				_motor->FOC.base.FOCAngle += _motor->hfi.HFI_test_increment;
				_motor->hfi.HFI_accu += magnitude45;
				_motor->hfi.HFI_count += 1;
			}
			#if 0 //Sometimes for investigation we want to just lock the angle, this is an easy bodge
							_motor->FOC.base.FOCAngle = 62000;
			#endif
				break;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case HFI_TYPE_D:
			if(_motor->hfi.inject_high_low_now ==1){
			  _motor->hfi.Vd_injectionV = +_motor->meas.hfi_voltage;
			}else{
			  _motor->hfi.Vd_injectionV = -_motor->meas.hfi_voltage;
			}
			if(_motor->hfi.didq.q>1.0f){_motor->hfi.didq.q = 1.0f;}
			if(_motor->hfi.didq.q<-1.0f){_motor->hfi.didq.q = -1.0f;}
			intdidq.q = (intdidq.q + 0.1f*_motor->hfi.didq.q);
			if(intdidq.q>10){intdidq.q=10;}
			if(intdidq.q<-10){intdidq.q=-10;}
			_motor->FOC.base.FOCAngle += (int)(250.0f*_motor->hfi.IIR[1] + 10.50f*intdidq.q)*_motor->hfi.d_polarity;
		break;
		case HFI_TYPE_SPECIAL:
			__NOP();
			if(_motor->hfi.inject_high_low_now ==1){
			  _motor->hfi.Vd_injectionV = _motor->hfi.special_injectionVd;
			  _motor->hfi.Vq_injectionV = _motor->hfi.special_injectionVq;
			}else{
				_motor->hfi.Vd_injectionV = -_motor->hfi.special_injectionVd;
				_motor->hfi.Vq_injectionV = -_motor->hfi.special_injectionVq;			}
		break;
	}
  }else {
	  _motor->hfi.Vd_injectionV = 0.0f;
	  _motor->hfi.Vq_injectionV = 0.0f;
  }
}

void ToggleHFI(MESC_motor_typedef *_motor) {
	if(((_motor->FOC.base.Vdq.q-_motor->FOC.base.Idq_smoothed.q*_motor->m.R) > _motor->hfi.HFI_toggle_voltage)||((_motor->FOC.base.Vdq.q-_motor->FOC.base.Idq_smoothed.q*_motor->m.R) < -_motor->hfi.HFI_toggle_voltage)||(_motor->MotorSensorMode==MOTOR_SENSOR_MODE_HALL)){
		_motor->hfi.inject = 0;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
	} else if(((_motor->FOC.base.Vdq.q-_motor->FOC.base.Idq_smoothed.q*_motor->m.R) < (_motor->hfi.HFI_toggle_voltage-1.0f))&&((_motor->FOC.base.Vdq.q-_motor->FOC.base.Idq_smoothed.q*_motor->m.R) > -(_motor->hfi.HFI_toggle_voltage-1.0f)) &&(_motor->HFIType !=HFI_TYPE_NONE)){
		_motor->hfi.HFI_int_err = _motor->FOC.base.PLL_int;
		_motor->hfi.inject = 1;
		_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH*0.1f;
	}
}

void SlowHFI(MESC_motor_typedef *_motor) {
	/////////////Set and reset the HFI////////////////////////
		switch(_motor->HFIType){
			case HFI_TYPE_45:
			ToggleHFI(_motor);
				if(_motor->hfi.inject==1){
					//static int no_q;
					if(_motor->hfi.was_last_tracking==1){
						if(_motor->hfi.HFI_countdown>1){
							_motor->hfi.HFI45_mod_didq = _motor->hfi.HFI_accu / _motor->hfi.HFI_count;
							_motor->hfi.HFI_Gain = 5000.0f/_motor->hfi.HFI45_mod_didq;
							_motor->hfi.was_last_tracking = 0;
						}else{
							_motor->hfi.HFI_test_increment = 65536 * SLOW_LOOP_FREQUENCY / _motor->FOC.base.pwm_frequency;
							_motor->hfi.HFI_countdown++;
						}
					}else{
						_motor->hfi.HFI_countdown = 0;
						_motor->hfi.HFI_count = 0;
						_motor->hfi.HFI_accu = 0.0f;
					}
				}
			break;

			case HFI_TYPE_D:
				ToggleHFI(_motor);
				if(_motor->hfi.inject==1){
					if(_motor->hfi.HFI_countdown==3){
						_motor->FOC.base.Idq_req.d = HFI_TEST_CURRENT;
						_motor->FOC.base.Idq_req.q=0.0f;//Override the inputs to set Q current to zero
					}else if(_motor->hfi.HFI_countdown==2){
						_motor->hfi.Ldq_now_dboost[0] = _motor->hfi.IIR[0]; //Find the effect of d-axis current
						_motor->FOC.base.Idq_req.d = 1.0f;
						_motor->FOC.base.Idq_req.q=0.0f;
					}else if(_motor->hfi.HFI_countdown == 1){
						_motor->FOC.base.Idq_req.d = -HFI_TEST_CURRENT;
						_motor->FOC.base.Idq_req.q=0.0f;
					}else if(_motor->hfi.HFI_countdown == 0){
						_motor->hfi.Ldq_now[0] = _motor->hfi.IIR[0];//_motor->hfi.Vd_injectionV;
						_motor->FOC.base.Idq_req.d = 0.0f;
					if(_motor->hfi.Ldq_now[0]>_motor->hfi.Ldq_now_dboost[0]){_motor->FOC.base.FOCAngle+=32768;}
					_motor->hfi.HFI_countdown = 200;
					}
					_motor->hfi.HFI_countdown--;
				}
			break;

			case HFI_TYPE_NONE:
				_motor->hfi.inject = 0;
//				_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
			break;
			case HFI_TYPE_SPECIAL:
				ToggleHFI(_motor);
			break;
		}
}

static float dinductance, qinductance;

float detectHFI(MESC_motor_typedef *_motor) {
	  ///Try out a new detection routine
#if 1

	_motor->meas.previous_HFI_type = _motor->HFIType;
	_motor->HFIType = HFI_TYPE_D;
	input_vars.UART_req = 0.25f;
	int a = 0;
	dinductance = 0;
	qinductance = 0;
	while(a<1000){
		a++;
		_motor->HFIType = HFI_TYPE_D;
		dinductance = dinductance + _motor->hfi.didq.d;
		HAL_Delay(0);
		//input_vars.input_options = 0b
	}
	dinductance = dinductance/1000.0f;
	//dinductance = motor1.FOC.pwm_period*motor1.hfi.Vd_injectionV/(motor1.Conv.Vbus*dinductance);
	//Vdt/di = L
	_motor->hfi.d_polarity = -1;
	a=0;
	while(a<1000){
		a++;
		_motor->HFIType = HFI_TYPE_D;
		qinductance = qinductance + _motor->hfi.didq.d;
		HAL_Delay(0);
		//input_vars.input_options = 0b
	}
	qinductance = qinductance/1000.0f; //Note that this is not yet an inductance, but an inverse of inductance*voltage
	_motor->hfi.HFI45_mod_didq = sqrtf(qinductance*qinductance+dinductance*dinductance);
	_motor->hfi.HFI_Gain = 5000.0f/_motor->hfi.HFI45_mod_didq; //Magic numbers that seem to work
	input_vars.UART_req = 0.0f;
	_motor->hfi.d_polarity = 1;

	_motor->HFIType = _motor->meas.previous_HFI_type;

	return _motor->hfi.HFI45_mod_didq;

#endif
}
