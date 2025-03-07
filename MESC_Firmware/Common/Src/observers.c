#include "observers.h"
#include "position.h"
#include "motorinstance.h"
#include "interrupt_adc.h"
#include <math.h>


/////////////////////////////////////////////////////////////////////////////
// SENSORLESS IMPLEMENTATION//////////////////////////////////////////////////
float fluxa, fluxb, fluxd, fluxq, fbd, fbq;
void flux_observer_V2(MESC_motor_typedef *_motor)
{
	/*	Inspired by the dq reference frame use of Alex Evers, author of the UniMoC
		This observer will attempt to track flux in the dq frame with the following actions:
		1) Carry out the fluxa = integral(Va-Rxia) and fluxb = integral(Vb-Rxib)
		2) Magically remove the inductive fluxes from the integral (subtract inverse park transform of LqIq and LdId?) and Apply atan2 to calculate the angle

		3) Convert the fluxes  to dq frame via park
		4) Modify with the subtraction of Lqxiq and Ldxid respectively
		5) Apply the feedback based on the deviation from reference fluxes
			5)a) From here, could get feedback for an observer in dq frame?
		6) Modify with the addition of Lqxiq and Ldxid respectively (inverse of 4)
		7) Convert fluxes from dq frame to ab frame overwriting fluxa and fluxb

	*/
	fluxa = fluxa + (_motor->FOC.Vab.a - _motor->FOC.Iab.a * _motor->m.R) * _motor->FOC.pwm_period;
	fluxb = fluxb + (_motor->FOC.Vab.b - _motor->FOC.Iab.b * _motor->m.R) * _motor->FOC.pwm_period;

	// Park transform
	fluxd = _motor->FOC.sincosangle.cos * fluxa +
			_motor->FOC.sincosangle.sin * fluxb;
	fluxq = _motor->FOC.sincosangle.cos * fluxb -
			_motor->FOC.sincosangle.sin * fluxa;

	// This is not part of the final version
	if (fluxa > _motor->FOC.flux_observed)
	{
		fluxa = _motor->FOC.flux_observed;
	}
	if (fluxa < -_motor->FOC.flux_observed)
	{
		fluxa = -_motor->FOC.flux_observed;
	}
	if (fluxb > _motor->FOC.flux_observed)
	{
		fluxb = _motor->FOC.flux_observed;
	}
	if (fluxb < -_motor->FOC.flux_observed)
	{
		fluxb = -_motor->FOC.flux_observed;
	}
}

float La_last, Lb_last;
void flux_observer(MESC_motor_typedef *_motor)
{
	// flux_observer_V2(_motor); //For testing comparison, running both at the same time
	//  LICENCE NOTE REMINDER:
	//  This work deviates slightly from the BSD 3 clause licence.
	//  The work here is entirely original to the MESC FOC project, and not based
	//  on any appnotes, or borrowed from another project. This work is free to
	//  use, as granted in BSD 3 clause, with the exception that this note must
	//  be included in where this code is implemented/modified to use your
	//  variable names, structures containing variables or other minor
	//  rearrangements in place of the original names I have chosen, and credit
	//  to David Molony as the original author must be noted.

	// With thanks to C0d3b453 for generally keeping this compiling and Elwin
	// for producing data comparing the output to a 16bit encoder.

#ifndef DONT_USE_FLUX_LINKAGE_OBSERVER
	// Variant of the flux linkage observer created by/with Benjamin Vedder to
	// eliminate the need to accurately know the flux linked motor parameter.
	// This may be useful when approaching saturation; currently unclear but
	// definitely makes setup less critical.
	// It basically takes the normal of the flux linkage at any time and
	// changes the flux limits accordingly, ignoring using a sqrt for computational efficiency
	float flux_linked_norm = _motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b * _motor->FOC.flux_b;
	float flux_err = flux_linked_norm - _motor->FOC.flux_observed * _motor->FOC.flux_observed;
	_motor->FOC.flux_observed = _motor->FOC.flux_observed + _motor->m.flux_linkage_gain * flux_err;
	if (_motor->FOC.flux_observed > _motor->m.flux_linkage_max)
	{
		_motor->FOC.flux_observed = _motor->m.flux_linkage_max;
	}
	if (_motor->FOC.flux_observed < _motor->m.flux_linkage_min)
	{
		_motor->FOC.flux_observed = _motor->m.flux_linkage_min;
	}
#endif
	// This is the actual observer function.
	// We are going to integrate Va-Ri and clamp it positively and negatively
	// the angle is then the arctangent of the integrals shifted 180 degrees
#ifdef USE_SALIENT_OBSERVER
	float La, Lb;
	getLabFast(_motor->FOC.FOCAngle, _motor->m.L_D, _motor->m.L_QD, &La, &Lb);

	_motor->FOC.flux_a = _motor->FOC.flux_a +
						 (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a) * _motor->FOC.pwm_period -
						 La * (_motor->FOC.Iab.a - _motor->FOC.Ia_last) - // Salient inductance NOW
						 _motor->FOC.Iab.a * (La - La_last);			  // Differential of phi = Li -> Ldi/dt+idL/dt
	_motor->FOC.flux_b = _motor->FOC.flux_b +
						 (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b) * _motor->FOC.pwm_period -
						 Lb * (_motor->FOC.Iab.b - _motor->FOC.Ib_last) -
						 _motor->FOC.Iab.b * (Lb - Lb_last);
	// Store the inductances
	La_last = La;
	Lb_last = Lb;
#else
	_motor->FOC.flux_a =
		_motor->FOC.flux_a + (_motor->FOC.Vab.a - _motor->m.R * _motor->FOC.Iab.a) * _motor->FOC.pwm_period -
		_motor->m.L_D * (_motor->FOC.Iab.a - _motor->FOC.Ia_last);
	_motor->FOC.flux_b =
		_motor->FOC.flux_b + (_motor->FOC.Vab.b - _motor->m.R * _motor->FOC.Iab.b) * _motor->FOC.pwm_period -
		_motor->m.L_D * (_motor->FOC.Iab.b - _motor->FOC.Ib_last);
#endif
	// Store the currents
	_motor->FOC.Ia_last = _motor->FOC.Iab.a;
	_motor->FOC.Ib_last = _motor->FOC.Iab.b;

#ifdef USE_NONLINEAR_OBSERVER_CENTERING
	/// Try directly applying the centering using the same method as the flux linkage observer
	float err = _motor->FOC.flux_observed * _motor->FOC.flux_observed - _motor->FOC.flux_a * _motor->FOC.flux_a - _motor->FOC.flux_b * _motor->FOC.flux_b;
	_motor->FOC.flux_b = _motor->FOC.flux_b + err * _motor->FOC.flux_b * _motor->m.non_linear_centering_gain;
	_motor->FOC.flux_a = _motor->FOC.flux_a + err * _motor->FOC.flux_a * _motor->m.non_linear_centering_gain;
#endif
#ifdef USE_CLAMPED_OBSERVER_CENTERING
	if (_motor->FOC.flux_a > _motor->FOC.flux_observed)
	{
		_motor->FOC.flux_a = _motor->FOC.flux_observed;
	}
	if (_motor->FOC.flux_a < -_motor->FOC.flux_observed)
	{
		_motor->FOC.flux_a = -_motor->FOC.flux_observed;
	}
	if (_motor->FOC.flux_b > _motor->FOC.flux_observed)
	{
		_motor->FOC.flux_b = _motor->FOC.flux_observed;
	}
	if (_motor->FOC.flux_b < -_motor->FOC.flux_observed)
	{
		_motor->FOC.flux_b = -_motor->FOC.flux_observed;
	}
#endif

	if (_motor->hfi.do_injection == 0)
	{
		_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
	}

#ifdef TRACK_ENCODER_OBSERVER_ERROR
	// This does not apply the encoder angle,
	// It tracks the difference between the encoder and the observer.
	_motor->FOC.enc_obs_angle = _motor->FOC.FOCAngle - _motor->FOC.enc_angle;
#endif
}



/////////////////////////////////////////////////////////////////////////////
////////Hall Sensor Implementation///////////////////////////////////////////

void hallAngleEstimator(MESC_motor_typedef *_motor)
{	// Implementation using the mid point of the hall
	// sensor angles, which should be much more
	// reliable to generate that the edges

	if (_motor->hall.current_hall_state != _motor->hall.last_hall_state)
	{
		_motor->FOC.hall_update = 1;
		if (_motor->hall.current_hall_state == 0)
		{
			_motor->MotorState = MOTOR_STATE_ERROR;
			MotorError = MOTOR_ERROR_HALL0;
		}
		else if (_motor->hall.current_hall_state == 7)
		{
			_motor->MotorState = MOTOR_STATE_ERROR;
			MotorError = MOTOR_ERROR_HALL7;
		}
		//////////Implement the Hall table here, but the vector can be dynamically
		/// created/filled by another function/////////////
		_motor->hall.current_hall_angle = _motor->m.hall_table[_motor->hall.current_hall_state - 1][2];

		// Calculate Hall error

		uint16_t a;
		if ((a = _motor->hall.current_hall_angle - _motor->hall.last_hall_angle) < 32000) // Forwards
		{
			_motor->hall.hall_error =
				_motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][0];
			_motor->hall.dir = 1.0f;
			// _motor->FOC.HallAngle = _motor->FOC.HallAngle - 5460;
		}
		else // Backwards
		{
			_motor->hall.hall_error =
				_motor->FOC.FOCAngle - _motor->m.hall_table[_motor->hall.current_hall_state - 1][1];
			_motor->hall.dir = -1.0f;
			// _motor->FOC.HallAngle = _motor->FOC.HallAngle + 5460;
		}
		if (_motor->hall.hall_error > 32000)
		{
			_motor->hall.hall_error = _motor->hall.hall_error - 65536;
		}
		if (_motor->hall.hall_error < -32000)
		{
			_motor->hall.hall_error = _motor->hall.hall_error + 65536;
		}
	}
}

void angleObserver(MESC_motor_typedef *_motor)
{
	// This function should take the available data (hall change, BEMF crossing
	// etc...) and process it with a PLL type mechanism
	if (_motor->FOC.hall_update == 1)
	{
		_motor->FOC.hall_update = 0;
		_motor->hall.last_observer_period = _motor->hall.ticks_since_last_observer_change;
		float one_on_ticks = (1.0f / _motor->hall.ticks_since_last_observer_change);
		_motor->hall.one_on_last_observer_period =
			(4.0f * _motor->hall.one_on_last_observer_period + (one_on_ticks)) * 0.2f; // ;
		_motor->hall.angle_step =
			(4.0f * _motor->hall.angle_step +
			 (one_on_ticks)*_motor->m.hall_table[_motor->hall.last_hall_state - 1][3]) *
			0.2f;

		// Reset the counters, track the previous state
		_motor->hall.last_hall_state = _motor->hall.current_hall_state;
		_motor->hall.last_hall_angle = _motor->hall.current_hall_angle;
		_motor->hall.ticks_since_last_observer_change = 0;
	}

	// Run the counter
	_motor->hall.ticks_since_last_observer_change = _motor->hall.ticks_since_last_observer_change + 1;

	if (_motor->hall.ticks_since_last_observer_change <= 2.0f * _motor->hall.last_observer_period)
	{
		/*      _motor->FOC.FOCAngle = _motor->FOC.FOCAngle + (uint16_t)(dir*angle_step
		   + one_on_last_observer_period * (-0.9f * hall_error)); //Does not
		   work...
			 //Why?
   */
		if (_motor->hall.dir > 0)
		{ // Apply a gain to the error as well as the feed forward
			// from the last hall period. Gain of 0.9-1.1 seems to work
			// well when using corrected hall positions and spacings
			_motor->FOC.FOCAngle =
				_motor->FOC.FOCAngle +
				(uint16_t)(_motor->hall.angle_step - _motor->hall.one_on_last_observer_period * _motor->hall.hall_error);
			// one_on_last_observer_period * (-0.2f * hall_error));
		}
		else if (_motor->hall.dir < 0.0f)
		{
			_motor->FOC.FOCAngle =
				_motor->FOC.FOCAngle +
				(uint16_t)(-_motor->hall.angle_step +
						   _motor->hall.one_on_last_observer_period * (-0.9f * _motor->hall.hall_error));
			// Also does not work,
			// Why??
			_motor->FOC.FOCAngle =
				_motor->FOC.FOCAngle -
				(uint16_t)(_motor->hall.angle_step +
						   _motor->hall.one_on_last_observer_period * (0.2f * _motor->hall.hall_error));
		}
	}
	if (_motor->hall.ticks_since_last_observer_change > 1500.0f)
	{
		_motor->hall.ticks_since_last_observer_change = 1500.0f;
		_motor->hall.last_observer_period = 1500.0f; //(ticks_since_last_hall_change);
		_motor->hall.one_on_last_observer_period =
			1.0f / _motor->hall.last_observer_period; // / ticks_since_last_hall_change;
		_motor->FOC.FOCAngle = _motor->hall.current_hall_angle;
	}
}


// LR observer. WIP function that works by injecting a
// low frequency Id signal into the PID input and observing the change in Vd and Vq
// Does not work too well, requires some care in use.
// Original work to MESC project.
float Vd_obs_high, Vd_obs_low, R_observer, Vq_obs_high, Vq_obs_low, L_observer, Last_eHz, LR_collect_count;
float Vd_obs_high_filt, Vd_obs_low_filt, Vq_obs_high_filt, Vq_obs_low_filt;
static int plusminus = 1;

void LRObserver(MESC_motor_typedef *_motor)
{
	if ((fabsf(_motor->FOC.eHz) > 0.005f * _motor->FOC.pwm_frequency) && (_motor->hfi.do_injection == 0))
	{

		R_observer = (Vd_obs_high_filt - Vd_obs_low_filt) / (2.0f * LR_OBS_CURRENT);
		L_observer = (Vq_obs_high_filt - Vq_obs_low_filt - 6.28f * (_motor->FOC.eHz - Last_eHz) * _motor->FOC.flux_observed) / (2.0f * LR_OBS_CURRENT * 6.28f * _motor->FOC.eHz);

		if (plusminus == 1)
		{
			plusminus = -1;
			Vd_obs_low_filt = Vd_obs_low / LR_collect_count;
			Vq_obs_low_filt = Vq_obs_low / LR_collect_count;
			_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d + 1.0f * LR_OBS_CURRENT;
			Vd_obs_low = 0;
			Vq_obs_low = 0;
		}
		else if (plusminus == -1)
		{
			plusminus = 1;
			Vd_obs_high_filt = Vd_obs_high / LR_collect_count;
			Vq_obs_high_filt = Vq_obs_high / LR_collect_count;
			_motor->FOC.Idq_req.d = _motor->FOC.Idq_req.d - 1.0f * LR_OBS_CURRENT;
			Vd_obs_high = 0;
			Vq_obs_high = 0;
		}
		Last_eHz = _motor->FOC.eHz;
		LR_collect_count = 0; // Reset this after doing the calcs
	}
#if 0
	  	float Rerror = R_observer-_motor->m.R;
	  	float Lerror = L_observer-_motor->m.L_D;
	  	//Apply the correction excluding large changes
	  	if(fabs(Rerror)<0.1f*_motor->m.R){
	  		_motor->m.R = _motor->m.R+0.1f*Rerror;
	  	}else if(fabs(Rerror)<0.5f*_motor->m.R){
	  		_motor->m.R = _motor->m.R+0.001f*Rerror;
	  	}
	  	if(fabs(Lerror)<0.1f*_motor->m.L_D){
	  		_motor->m.L_D = _motor->m.L_D+0.1f*Lerror;
	  		_motor->m.L_Q = _motor->m.L_Q +0.1f*Lerror;
	  	}else if(fabs(Lerror)<0.5f*_motor->m.L_D){
	  		_motor->m.L_D = _motor->m.L_D+0.001f*Lerror;
	  		_motor->m.L_Q = _motor->m.L_Q +0.001f*Lerror;
	  	}

#endif
}

void LRObserverCollect(MESC_motor_typedef *_motor)
{
	LR_collect_count++;
	if ((fabsf(_motor->FOC.eHz) > 0.005f * _motor->FOC.pwm_frequency) && (_motor->hfi.do_injection == 0))
	{
		if (plusminus == 1)
		{
			Vd_obs_low = Vd_obs_low + _motor->FOC.Vdq.d;
			Vq_obs_low = Vq_obs_low + _motor->FOC.Vdq.q;
		}
		if (plusminus == -1)
		{
			Vd_obs_high = Vd_obs_high + _motor->FOC.Vdq.d;
			Vq_obs_high = Vq_obs_high + _motor->FOC.Vdq.q;
		}
	}
}

void HallFluxMonitor(MESC_motor_typedef *_motor)
{
	if (fabsf(_motor->FOC.Vdq.q) > MIN_HALL_FLUX_VOLTS)
	{ // Are we actually spinning at a reasonable pace?
		if ((_motor->hall.current_hall_state > 0) && (_motor->hall.current_hall_state < 7))
		{
			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] =
				0.999f * _motor->m.hall_flux[_motor->hall.current_hall_state - 1][0] +
				0.001f * _motor->FOC.flux_a;
			// take a slow average of the alpha flux linked and store it for later preloading
			// the observer during very low speed conditions. There is a slight bias towards
			// later values of flux linked, which is probably good.
			_motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] =
				0.999f * _motor->m.hall_flux[_motor->hall.current_hall_state - 1][1] +
				0.001f * _motor->FOC.flux_b;
		}
		_motor->FOC.hall_initialised = 1;
	}
}
