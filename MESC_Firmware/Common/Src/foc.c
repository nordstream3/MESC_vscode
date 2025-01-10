#include "foc.h"
#include "motorinstance.h"
#include "conversions.h"
#include "sin_lut.h"
#include "measurements.h"
#include <math.h>

#define foc _motor->FOC

extern TIM_HandleTypeDef htim1;

// fast_atan2 based on https://math.stackexchange.com/a/1105038/81278
// Via Odrive project
// https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/utils.cpp
// This function is MIT licenced, copyright Oskar Weigl/Odrive Robotics
// The origin for Odrive atan2 is public domain. Thanks to Odrive for making
// it easy to borrow.
float min(float lhs, float rhs) { return (lhs < rhs) ? lhs : rhs; }
float max(float lhs, float rhs) { return (lhs > rhs) ? lhs : rhs; }

float fast_atan2(float y, float x)
{
	// a := min (|x|, |y|) / max (|x|, |y|)
	float abs_y = fabsf(y);
	float abs_x = fabsf(x);
	// inject FLT_MIN in denominator to avoid division by zero
	float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
	// s := a * a
	float s = a * a;
	// r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
	float r =
		((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
	// if |y| > |x| then r := 1.57079637 - r
	if (abs_y > abs_x)
		r = 1.57079637f - r;
	// if x < 0 then r := 3.14159274 - r
	if (x < 0.0f)
		r = 3.14159274f - r;
	// if y < 0 then r := -r
	if (y < 0.0f)
		r = -r;

	return r;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FOC PID algorithms
//////////////////////////////////////////////////////////////////////////////////////////

void MESCFOC(MESC_motor_typedef *_motor)
{

	// Here we are going to do a PID loop to control the dq currents, converting
	// Idq into Vdq Calculate the errors
	static MESCiq_s Idq_err;
	Idq_err.q = (foc.Idq_req.q - foc.Idq.q) * foc.Iq_pgain;
#if defined(USE_FIELD_WEAKENING) || defined(USE_FIELD_WEAKENINGV2)
	if ((foc.FW_current < foc.Idq_req.d) && (_motor->MotorState == MOTOR_STATE_RUN))
	{ // Field weakenning is -ve, but there may already be d-axis from the MTPA
		Idq_err.d = (foc.FW_current - foc.Idq.d) * foc.Id_pgain;
	}
	else
	{
		Idq_err.d = (foc.Idq_req.d - foc.Idq.d) * foc.Id_pgain;
	}
#else
	Idq_err.d = (foc.Idq_req.d - foc.Idq.d) * foc.Id_pgain;
	// if we do not use the field weakening controller, we still want to control the d axis current...
#endif

	// Integral error
	foc.Idq_int_err.d =
		foc.Idq_int_err.d + foc.Id_igain * Idq_err.d * foc.pwm_period;
	foc.Idq_int_err.q =
		foc.Idq_int_err.q + foc.Iq_igain * Idq_err.q * foc.pwm_period;
	// Apply the integral gain at this stage to enable bounding it

	// Apply the PID, and potentially smooth the output for noise - sudden
	// changes in VDVQ may be undesirable for some motors. Integral error is
	// pre-bounded to avoid integral windup, proportional gain needs to have
	// effect even at max integral to stabilise and avoid trips
	foc.Vdq.d = Idq_err.d + foc.Idq_int_err.d;
	foc.Vdq.q = Idq_err.q + foc.Idq_int_err.q;

	// Bounding final output

#if defined(USE_SQRT_CIRCLE_LIM)
	float Vmagnow2 = foc.Vdq.d * foc.Vdq.d + foc.Vdq.q * foc.Vdq.q;
	// Check if the vector length is greater than the available voltage
	foc.Voltage = sqrtf(Vmagnow2);
	if (foc.Voltage > foc.Vmag_max)
	{
		// float Vmagnow = sqrtf(Vmagnow2);
		float one_on_Vmagnow = 1.0f / foc.Voltage;
		float one_on_VmagnowxVmagmax = foc.Vmag_max * one_on_Vmagnow;
		foc.Vdq.d = foc.Vdq.d * one_on_VmagnowxVmagmax;
		foc.Vdq.q = foc.Vdq.q * one_on_VmagnowxVmagmax;
		foc.Idq_int_err.d = foc.Idq_int_err.d * one_on_VmagnowxVmagmax;
		foc.Idq_int_err.q = foc.Idq_int_err.q * one_on_VmagnowxVmagmax;
#ifdef USE_FIELD_WEAKENINGV2
		// Preferable to use FWV2 with the D axis circle limiter,
		// this allows the D current to ramp all the way to max, whereas
		// the linear sqrt circle limiter is overcome by large q axis voltage demands
		// Closed loop field weakenning that works by only applying D axis current in the case where there is no duty left.
		// Seems very effective at increasing speed with good stability and maintaining max torque.
		foc.FW_current = 0.99f * foc.FW_current - 0.01f * foc.FW_curr_max;
		// Exponentially tend towards the max FW current
	}
	else
	{
		foc.FW_current = 1.01f * foc.FW_current + 0.0101f * foc.FW_curr_max;
	} // Unroll the exponential ramp up, with a small extra term to ensure we do not saturate the float
	if (foc.FW_current > foc.Idq_req.d)
	{
		foc.FW_current = foc.Idq_req.d;
	}
	if (foc.FW_current < -foc.FW_curr_max)
	{
		foc.FW_current = -foc.FW_curr_max;
	}
#else
	} // Just close the bracket
#endif
#elif defined(USE_SQRT_CIRCLE_LIM_VD)
	// Circle limiter that favours Vd, similar to used in VESC, and as an option in ST firmware.for torque
	// This method was primarily designed for induction motors, where the d axis is required to
	// make the magnetic field for torque. Nevertheless, this finds application at extreme currents and
	// during field weakening.
	// Latent concerns about the usual implementation that allows ALL the voltage to be
	// assigned to Vd becoming unstable as the angle relative to the rotor exceeds 45 degrees
	// due to rapidly collapsing q-axis voltage. Therefore, this option will be allowed, but
	//  with a limit of voltage angle 60degrees (sin60 = 0.866) from the rotor.

	if (foc.Vdq.d < -0.866f * foc.Vmag_max)
	{														// Negative values of Vd - Normally Vd is -ve since it is driving field advance
		foc.Vdq.d = -0.866f * foc.Vmag_max; // Hard clamp the Vd
		if (foc.Idq_int_err.d < foc.Vdq.d)
		{
			foc.Idq_int_err.d = foc.Vdq.d; // Also clamp the integral to stop windup
		}
	}
	else if (foc.Vdq.d > 0.866f * foc.Vmag_max)
	{													   // Positive values of Vd
		foc.Vdq.d = 0.866f * foc.Vmag_max; // Hard clamp the Vd
		if (foc.Idq_int_err.d > foc.Vdq.d)
		{
			foc.Idq_int_err.d = foc.Vdq.d; // Also clamp the integral to stop windup
		}
	}

	// Now we take care of the overall length of the voltage vector
	float Vmagnow2 = foc.Vdq.d * foc.Vdq.d + foc.Vdq.q * foc.Vdq.q;
	foc.Voltage = sqrtf(Vmagnow2);
	if (foc.Voltage > foc.Vmag_max)
	{
		foc.Voltage = foc.Vmag_max;
		if (foc.Vdq.q > 0.0f)
		{ // Positive Vq
			foc.Vdq.q = sqrtf(foc.Vmag_max2 - foc.Vdq.d * foc.Vdq.d);
			if (foc.Idq_int_err.q > foc.Vdq.q)
			{
				foc.Idq_int_err.q = foc.Vdq.q;
			}
		}
		else
		{ // Negative Vq
			foc.Vdq.q = -sqrtf(foc.Vmag_max2 - foc.Vdq.d * foc.Vdq.d);
			if (foc.Idq_int_err.q < foc.Vdq.q)
			{
				foc.Idq_int_err.q = foc.Vdq.q;
			}
		}
	}
#ifdef USE_FIELD_WEAKENINGV2
	if (foc.Voltage > 0.95f * foc.Vmag_max)
	{
		// Closed loop field weakenning that works by only applying D axis current in the case where there is no duty left.
		// Added extra comparison statement to allow 5% excess duty which gives some headroom for the q axis PI control
		// Seems very effective at increasing speed with good stability and maintaining max torque.
		foc.FW_current = 0.99f * foc.FW_current - 0.01f * foc.FW_curr_max;
		// Exponentially tend towards the max FW current
	}
	else
	{
		foc.FW_current = 1.01f * foc.FW_current + 0.0101f * foc.FW_curr_max;
	} // Exponentially diverge from the FW current. Note that this exponential implemented opposite to the ramp up!
	if (foc.FW_current > foc.Idq_req.d)
	{
		foc.FW_current = foc.Idq_req.d;
	}
	if (foc.FW_current < -foc.FW_curr_max)
	{
		foc.FW_current = -foc.FW_curr_max;
	}
#endif
#else
	// Fixed Vd and Vq limits.
	//  These limits are experimental, but result in close to 100% modulation.
	//  Since Vd and Vq are orthogonal, limiting Vd is not especially helpful
	//  in reducing overall voltage magnitude, since the relation
	//  Vout=(Vd^2+Vq^2)^0.5 results in Vd having a small effect. Vd is
	//  primarily used to drive the resistive part of the field; there is no
	//  BEMF pushing against Vd and so it does not scale with RPM (except for
	//  cross coupling).

	// Bounding integral
	if (foc.Idq_int_err.d > foc.Vdint_max)
	{
		foc.Idq_int_err.d = foc.Vdint_max;
	}
	if (foc.Idq_int_err.d < -foc.Vdint_max)
	{
		foc.Idq_int_err.d = -foc.Vdint_max;
	}
	if (foc.Idq_int_err.q > foc.Vqint_max)
	{
		foc.Idq_int_err.q = foc.Vqint_max;
	}
	if (foc.Idq_int_err.q < -foc.Vqint_max)
	{
		foc.Idq_int_err.q = -foc.Vqint_max;
	}
	// Bounding output
	if (foc.Vdq.d > foc.Vd_max)
		(foc.Vdq.d = foc.Vd_max);
	if (foc.Vdq.d < -foc.Vd_max)
		(foc.Vdq.d = -foc.Vd_max);
	if (foc.Vdq.q > foc.Vq_max)
		(foc.Vdq.q = foc.Vq_max);
	if (foc.Vdq.q < -foc.Vq_max)
		(foc.Vdq.q = -foc.Vq_max);
#endif
#ifdef USE_FIELD_WEAKENING
	// Calculate the module of voltage applied,
	Vmagnow2 = foc.Vdq.d * foc.Vdq.d + foc.Vdq.q * foc.Vdq.q; // Need to recalculate this since limitation has maybe been applied
	// Apply a linear slope from the threshold to the max module. Similar methodology to VESC, but run in fast loop
	// Step towards with exponential smoother
	if (Vmagnow2 > (foc.FW_threshold * foc.FW_threshold))
	{
		foc.FW_current = 0.95f * foc.FW_current +
								 0.05f * foc.FW_curr_max * foc.FW_multiplier *
									 (foc.FW_threshold - sqrtf(Vmagnow2));
	}
	else
	{									 // We are outside the FW region
		foc.FW_current *= 0.95f; // Ramp down a bit slowly
		if (foc.FW_current > 0.1f)
		{ // We do not allow positive field weakening current, and we want it to actually go to zero eventually
			foc.FW_current = 0.0f;
		}
	}
	// Apply the field weakening only if the additional d current is greater than the requested d current

#endif
}

static float mid_value = 0;
float top_value;
float bottom_value;
//uint16_t deadtime_comp = DEADTIME_COMP_V;

void writePWM(MESC_motor_typedef *_motor)
{
	float Vd, Vq;

	Vd = foc.Vdq.d + _motor->hfi.Vd_injectionV;
	Vq = foc.Vdq.q + _motor->hfi.Vq_injectionV;

	// Now we update the sin and cos values, since when we do the inverse
	// transforms, we would like to use the most up to date versions(or even the
	// next predicted version...)
#ifdef INTERPOLATE_V7_ANGLE
	if ((fabsf(foc.eHz) > 0.005f * foc.pwm_frequency) && (_motor->hfi.inject == 0))
	{
		// Only run it when there is likely to be good speed measurement stability and
		// actual utility in doing it. At low speed, there is minimal benefit, and
		// unstable speed estimation could make it worse.
		// Presently, this causes issues with openloop iteration, and effectively doubles the speed. TBC
		foc.FOCAngle = foc.FOCAngle + 0.5f * foc.PLL_int;
	}
#endif
	sin_cos_fast(foc.FOCAngle, &foc.sincosangle.sin, &foc.sincosangle.cos);

	// Inverse Park transform
	foc.Vab.a = foc.sincosangle.cos * Vd -
						foc.sincosangle.sin * Vq;
	foc.Vab.b = foc.sincosangle.sin * Vd +
						foc.sincosangle.cos * Vq;
#ifdef STEPPER_MOTOR // Skip inverse Clark

	_motor->mtimer->Instance->CCR1 = (uint16_t)(1.0f * foc.Vab_to_PWM * (foc.Vab.a) + foc.PWMmid);
	_motor->mtimer->Instance->CCR2 = (uint16_t)(-1.0f * foc.Vab_to_PWM * (foc.Vab.a) + foc.PWMmid);
	_motor->mtimer->Instance->CCR3 = (uint16_t)(1.0f * foc.Vab_to_PWM * (foc.Vab.b) + foc.PWMmid);
	_motor->mtimer->Instance->CCR4 = (uint16_t)(-1.0f * foc.Vab_to_PWM * (foc.Vab.b) + foc.PWMmid);

#else
	// Inverse Clark transform - power variant
	foc.inverterVoltage[0] = foc.Vab.a;
	foc.inverterVoltage[1] = -0.5f * foc.inverterVoltage[0];
	foc.inverterVoltage[2] = foc.inverterVoltage[1] - CONST_SQRT_3_R_2_F * foc.Vab.b;
	foc.inverterVoltage[1] = foc.inverterVoltage[1] + CONST_SQRT_3_R_2_F * foc.Vab.b;

	////////////////////////////////////////////////////////
	// SVPM implementation
	// Try to do this as a "midpoint clamp" where rather than finding the
	// lowest, we find the highest and lowest and subtract the middle
	top_value = foc.inverterVoltage[0];
	bottom_value = top_value;
	_motor->HighPhase = U;

	if (foc.inverterVoltage[1] > top_value)
	{
		top_value = foc.inverterVoltage[1];
		_motor->HighPhase = V;
	}
	if (foc.inverterVoltage[2] > top_value)
	{
		top_value = foc.inverterVoltage[2];
		_motor->HighPhase = W;
	}
	if (foc.inverterVoltage[1] < bottom_value)
	{
		bottom_value = foc.inverterVoltage[1];
	}
	if (foc.inverterVoltage[2] < bottom_value)
	{
		bottom_value = foc.inverterVoltage[2];
	}
	if (foc.Voltage < foc.V_3Q_mag_max)
	{
		_motor->HighPhase = N; // Trigger the full clark transform
	}
#ifdef SEVEN_SECTOR
	mid_value = foc.PWMmid -
				0.5f * foc.Vab_to_PWM * (top_value + bottom_value);

	////////////////////////////////////////////////////////
	// Actually write the value to the timer registers
	_motor->mtimer->Instance->CCR1 =
		(uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[0] + mid_value);
	_motor->mtimer->Instance->CCR2 =
		(uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[1] + mid_value);
	_motor->mtimer->Instance->CCR3 =
		(uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[2] + mid_value);

	// Dead time compensation
#ifdef DEADTIME_COMP
	// LICENCE NOTE:
	// This function deviates slightly from the BSD 3 clause licence.
	// The work here is entirely original to the MESC FOC project, and not based
	// on any appnotes, or borrowed from another project. This work is free to
	// use, as granted in BSD 3 clause, with the exception that this note must
	// be included in where this code is implemented/modified to use your
	// variable names, structures containing variables or other minor
	// rearrangements in place of the original names I have chosen, and credit
	// to David Molony as the original author must be noted.
	// The problem with dead time, is that it is essentially a voltage tie through the body diodes to VBus or ground, depending on the current direction.
	// If we know the direction of current, and the effective dead time length we can remove this error, by writing the corrected voltage.
	// This is observed to improve sinusoidalness of currents, but has a slight audible buzz
	// When the current is approximately zero, it is hard to resolve the direction, and therefore the compensation is ineffective.
	// However, no torque is generated when the current and voltage are close to zero, so no adverse performance except the buzz.
	if (_motor->Conv.Iu < -0.030f)
	{
		_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1 - foc.deadtime_comp;
	}
	if (_motor->Conv.Iv < -0.030f)
	{
		_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2 - foc.deadtime_comp;
	}
	if (_motor->Conv.Iw < -0.030f)
	{
		_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3 - foc.deadtime_comp;
	}
	if (_motor->Conv.Iu > -0.030f)
	{
		_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1 + foc.deadtime_comp;
	}
	if (_motor->Conv.Iv > -0.030f)
	{
		_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2 + foc.deadtime_comp;
	}
	if (_motor->Conv.Iw > -0.030f)
	{
		_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3 + foc.deadtime_comp;
	}

#endif
#else // Use 5 sector, bottom clamp implementation
	// ToDo, threshold for turning on sinusoidal modulation
	foc.inverterVoltage[0] = foc.inverterVoltage[0] - bottom_value;
	foc.inverterVoltage[1] = foc.inverterVoltage[1] - bottom_value;
	foc.inverterVoltage[2] = foc.inverterVoltage[2] - bottom_value;

	_motor->mtimer->Instance->CCR1 = (uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[0]);
	_motor->mtimer->Instance->CCR2 = (uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[1]);
	_motor->mtimer->Instance->CCR3 = (uint16_t)(foc.Vab_to_PWM * foc.inverterVoltage[2]);
#ifdef OVERMOD_DT_COMP_THRESHOLD
	// Concept here is that if we are close to the VBus max, we just do not turn the FET off.
	// Set CCRx to ARR, record how much was added, then next cycle, remove it from the count.
	// If the duty is still above the threshold, the CCR will still be set to ARR, until the duty request is sufficiently low...
	static int carryU, carryV, carryW;

	_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->CCR1 - carryU;
	_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->CCR2 - carryV;
	_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->CCR3 - carryW;
	carryU = 0;
	carryV = 0;
	carryW = 0;

	if (_motor->mtimer->Instance->CCR1 > (_motor->mtimer->Instance->ARR - OVERMOD_DT_COMP_THRESHOLD))
	{
		carryU = _motor->mtimer->Instance->ARR - _motor->mtimer->Instance->CCR1; // Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR1 = _motor->mtimer->Instance->ARR;
	}
	if (_motor->mtimer->Instance->CCR2 > (_motor->mtimer->Instance->ARR - OVERMOD_DT_COMP_THRESHOLD))
	{
		carryV = _motor->mtimer->Instance->ARR - _motor->mtimer->Instance->CCR2; // Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR2 = _motor->mtimer->Instance->ARR;
	}
	if (_motor->mtimer->Instance->CCR3 > (_motor->mtimer->Instance->ARR - OVERMOD_DT_COMP_THRESHOLD))
	{
		carryW = _motor->mtimer->Instance->ARR - _motor->mtimer->Instance->CCR3; // Save the amount we have overmodulated by
		_motor->mtimer->Instance->CCR3 = _motor->mtimer->Instance->ARR;
	}
#endif
#endif
#endif // End of #ifdef STEPPER_MOTOR
}

void MESCTrack(MESC_motor_typedef *_motor)
{
	// here we are going to do the clark and park transform of the voltages to
	// get the VaVb and VdVq These can be handed later to the observers and used
	// to set the integral terms

	// Accumulate the current offsets while there is no current (tri-stated)
	_motor->offset.Iu = 0.9999f * _motor->offset.Iu + 0.0001f * (float)_motor->Raw.Iu;
	_motor->offset.Iv = 0.9999f * _motor->offset.Iv + 0.0001f * (float)_motor->Raw.Iv;
	_motor->offset.Iw = 0.9999f * _motor->offset.Iw + 0.0001f * (float)_motor->Raw.Iw;

	// Clark transform
	foc.Vab.a =
		0.666f * (_motor->Conv.Vu -
				  0.5f * ((_motor->Conv.Vv) +
						  (_motor->Conv.Vw)));
	foc.Vab.b =
		0.666f *
		(CONST_SQRT_3_R_2_F * ((_motor->Conv.Vv) -
					   (_motor->Conv.Vw)));

	sin_cos_fast(foc.FOCAngle, &foc.sincosangle.sin, &foc.sincosangle.cos);

	// Park transform

	foc.Vdq.d = foc.sincosangle.cos * foc.Vab.a +
						foc.sincosangle.sin * foc.Vab.b;
	foc.Vdq.q = foc.sincosangle.cos * foc.Vab.b -
						foc.sincosangle.sin * foc.Vab.a;
	foc.Idq_int_err.q = foc.Vdq.q;
	foc.Idq_int_err.d = foc.Vdq.d;
}

float IacalcDS, IbcalcDS, VacalcDS, VbcalcDS, VdcalcDS, VqcalcDS, FLaDS, FLbDS, FLaDSErr, FLbDSErr;
uint16_t angleDS, angleErrorDSENC, angleErrorPhaseSENC, angleErrorPhaseDS, countdown_cycles;

void deadshort(MESC_motor_typedef *_motor)
{
	// LICENCE NOTE:
	// This function deviates slightly from the BSD 3 clause licence.
	// The work here is entirely original to the MESC FOC project, and not based
	// on any appnotes, or borrowed from another project. This work is free to
	// use, as granted in BSD 3 clause, with the exception that this note must
	// be included in where this code is implemented/modified to use your
	// variable names, structures containing variables or other minor
	// rearrangements in place of the original names I have chosen, and credit
	// to David Molony as the original author must be noted.

	// This "deadshort " function is an original idea (who knows, someone may have had it before) for finding the rotor angle
	// Concept is that when starting from spinning with no phase sensors or encoder, you need to know the angle and the voltages.
	// To achieve this, we simply short out the motor for a PWM period and allow the current to build up.
	// We can then calculate the voltage from V=Ldi/dt in the alpha beta reference frame
	// We can calculate the angle from the atan2 of the alpha beta voltages
	// With this angle, we can get Vd and Vq for preloading the PI controllers
	// We can also preload the flux observer with motor.motorflux*sin and motor.motorflux*cos terms

	static uint16_t countdown = 10;

	if (countdown == 1 || (((foc.Iab.a * foc.Iab.a + foc.Iab.b * foc.Iab.b) > DEADSHORT_CURRENT * DEADSHORT_CURRENT) && countdown < 9))
	{
		// Need to collect the ADC currents here
		generateBreak(_motor);
		// Calculate the voltages in the alpha beta phase...
		IacalcDS = foc.Iab.a;
		IbcalcDS = foc.Iab.b;
		VacalcDS = -_motor->m.L_D * foc.Iab.a / ((9.0f - (float)countdown) * foc.pwm_period);
		VbcalcDS = -_motor->m.L_D * foc.Iab.b / ((9.0f - (float)countdown) * foc.pwm_period);
		// Calculate the phase angle
		// TEST LINE angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768;// +16384;

		angleDS = (uint16_t)(32768.0f + 10430.0f * fast_atan2(VbcalcDS, VacalcDS)) - 32768 - 16384;
		// Shifting by 1/4 erev does not work for going backwards. Need to rethink.
		// Problem is, depending on motor direction, the sign of the voltage generated swaps for the same rotor position.
		// The atan2(flux linkages) is stable under this regime, but the same for voltage is not.
		foc.FOCAngle = angleDS; //
		sin_cos_fast(foc.FOCAngle, &foc.sincosangle.sin, &foc.sincosangle.cos);

		// Park transform it to get VdVq
		VdcalcDS = foc.sincosangle.cos * VacalcDS +
				   foc.sincosangle.sin * VbcalcDS;
		VqcalcDS = foc.sincosangle.cos * VbcalcDS -
				   foc.sincosangle.sin * VacalcDS;
		// Preloading the observer
		FLaDS = foc.flux_observed * foc.sincosangle.cos;
		FLbDS = foc.flux_observed * foc.sincosangle.sin;
		// Angle Errors for debugging
		angleErrorDSENC = angleDS - foc.enc_angle;
		// Do actual preloading
		foc.flux_a = FLaDS;
		foc.flux_b = FLbDS;
		foc.Ia_last = 0.0f;
		foc.Ib_last = 0.0f;
		foc.Idq_int_err.d = VdcalcDS;
		foc.Idq_int_err.q = VqcalcDS;
		// Next PWM cycle it  will jump to running state,
		MESCFOC(_motor);
		countdown_cycles = 9 - countdown;
		countdown = 1;
	}
	if (countdown > 10)
	{
		generateBreak(_motor);
		htim1.Instance->CCR1 = 50;
		htim1.Instance->CCR2 = 50;
		htim1.Instance->CCR3 = 50;
		// Preload the timer at mid
	}
	if (countdown <= 10 && countdown > 1)
	{
		htim1.Instance->CCR1 = 50;
		htim1.Instance->CCR2 = 50;
		htim1.Instance->CCR3 = 50;
		generateEnable(_motor);
	}
	if (countdown == 1)
	{
		countdown = 15; // We need at least a few cycles for the current to relax
						// to zero in case of rapid switching between states
		_motor->MotorState = MOTOR_STATE_RUN;
	}
	countdown--;
}

void SlowStartup(MESC_motor_typedef *_motor)
{
	switch (_motor->SLStartupSensor)
	{
	case STARTUP_SENSOR_HALL:
		if ((fabsf(foc.Vdq.q - _motor->m.R * foc.Idq_smoothed.q) < HALL_VOLTAGE_THRESHOLD) && (foc.hall_initialised) && (_motor->hall.current_hall_state > 0) && (_motor->hall.current_hall_state < 7))
		{
			foc.hall_start_now = 1;
		}
		else if ((fabsf(foc.Vdq.q - _motor->m.R * foc.Idq_smoothed.q) > HALL_VOLTAGE_THRESHOLD + 2.0f) || (_motor->hall.current_hall_state = 0) || (_motor->hall.current_hall_state > 6))
		{
			foc.hall_start_now = 0;
		}
		break;
	case STARTUP_SENSOR_PWM_ENCODER:
		if ((fabsf(foc.Vdq.q - _motor->m.R * foc.Idq_smoothed.q) < HALL_VOLTAGE_THRESHOLD) && (foc.encoder_OK))
		{
			foc.enc_start_now = 1;
		}
		else if ((fabsf(foc.Vdq.q - _motor->m.R * foc.Idq_smoothed.q) > HALL_VOLTAGE_THRESHOLD + 2.0f) || !(foc.encoder_OK))
		{
			foc.enc_start_now = 0;
		}
		break;
	case STARTUP_SENSOR_HFI:
		SlowHFI(_motor);
		break;
	default: // We are not using a startup mechanism
		foc.hall_start_now = 0;
		foc.enc_start_now = 0;
	}
}

void OLGenerateAngle(MESC_motor_typedef *_motor)
{
	//_motor->FOC.PLL_int = 0.5f*foc.openloop_step;
	foc.FOCAngle = foc.FOCAngle + foc.openloop_step;
	// ToDo
}

void RunMTPA(MESC_motor_typedef *_motor)
{
// Run MTPA (Field weakening seems to have to go in  the fast loop to be stable)
#ifdef USE_MTPA

	if (_motor->m.L_QD > 0.0f)
	{
		foc.id_mtpa = _motor->m.flux_linkage / (4.0f * _motor->m.L_QD) - sqrtf((_motor->m.flux_linkage * _motor->m.flux_linkage / (16.0f * _motor->m.L_QD * _motor->m.L_QD)) + foc.Idq_prereq.q * foc.Idq_prereq.q * 0.5f);
		if (fabsf(foc.Idq_prereq.q) > fabsf(foc.id_mtpa))
		{
			foc.iq_mtpa = sqrtf(foc.Idq_prereq.q * foc.Idq_prereq.q - foc.id_mtpa * foc.id_mtpa);
		}
		else
		{
			foc.iq_mtpa = 0.0f;
		}
		foc.Idq_prereq.d = foc.id_mtpa;
		if (foc.Idq_prereq.q > 0.0f)
		{
			foc.Idq_prereq.q = foc.iq_mtpa;
		}
		else
		{
			foc.Idq_prereq.q = -foc.iq_mtpa;
		}
	}

#endif
}
