#include "measurements.h"
#include "motorinstance.h"
#include "input_vars.h"
#include "conversions.h"
#include "hw_setup.h"
#include "observers.h"
#include <math.h>


MESCtest_s test_vals;

extern TIM_HandleTypeDef htim1;

// Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
// which should only be set by the hardware comparators, in the case of a
// shoot-through or other catastrophic event This function means that the
// timer can be left running, ADCs sampling etc which enables a recovery, or
// single PWM period break in which the backEMF can be measured directly
// This function needs implementing and testing before any high current or
// voltage is applied, otherwise... DeadFETs
void generateBreak(MESC_motor_typedef *_motor)
{
#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO << 16U; // Write the inverter enable pin low
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO << 16U; // Write the inverter enable pin low
#endif
	phU_Break(_motor);
	phV_Break(_motor);
	phW_Break(_motor);
}
void generateEnable(MESC_motor_typedef *_motor)
{
#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO; // Write the inverter enable pin high
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO; // Write the inverter enable pin high
#endif
	phU_Enable(_motor);
	phV_Enable(_motor);
	phW_Enable(_motor);
}

void measureResistance(MESC_motor_typedef *_motor)
{
	if (_motor->meas.PWM_cycles < 2)
	{
		_motor->meas.previous_HFI_type = _motor->HFIType;
		uint16_t half_ARR = htim1.Instance->ARR / 2;
		htim1.Instance->CCR1 = half_ARR;
		htim1.Instance->CCR2 = half_ARR;
		htim1.Instance->CCR3 = half_ARR;
		_motor->m.R = 0.001f;	   // Initialise with a very low value 1mR
		_motor->m.L_D = 0.000001f; // Initialise with a very low value 1uH
		_motor->m.L_Q = 0.000001f;
		calculateVoltageGain(_motor); // Set initial gains to enable MESCFOC to run
		calculateGains(_motor);
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
		_motor->FOC.Idq_req.d = _motor->meas.measure_current;
		_motor->FOC.Idq_req.q = 0.0f;
		_motor->FOC.FOCAngle = 0;

		_motor->hfi.inject = 0; // flag to not inject at SVPWM top

		MESCFOC(_motor);
		//      writePWM(_motor);

		_motor->meas.top_V = 0;
		_motor->meas.bottom_V = 0;
		_motor->meas.top_I = 0;
		_motor->meas.bottom_I = 0;
		_motor->meas.top_I_L = 0;
		_motor->meas.bottom_I_L = 0;
		_motor->meas.top_I_Lq = 0;
		_motor->meas.bottom_I_Lq = 0;

		_motor->meas.count_top = 0.0f;
		_motor->meas.count_bottom = 0.0f;
	}

	else if (_motor->meas.PWM_cycles < 35000)
	{ // Align the rotor for ~1 second
		_motor->FOC.Idq_req.d = _motor->meas.measure_current;
		_motor->FOC.Idq_req.q = 0.0f;

		_motor->hfi.inject = 0;
		MESCFOC(_motor);
		//      writePWM(_motor);
	}

	else if (_motor->meas.PWM_cycles < 40000)
	{ // Lower setpoint
		_motor->FOC.Idq_req.d = 0.20f * _motor->meas.measure_current;
		_motor->hfi.inject = 0;
		MESCFOC(_motor);
		//      writePWM(_motor);

		_motor->meas.bottom_V = _motor->meas.bottom_V + _motor->FOC.Vdq.d;
		_motor->meas.bottom_I = _motor->meas.bottom_I + _motor->FOC.Idq.d;
		_motor->meas.count_bottom++;
		_motor->meas.Vd_temp = _motor->FOC.Vdq.d * 1.0f; // Store the voltage required for the low setpoint, to
														 // use as an offset for the inductance
	}

	else if (_motor->meas.PWM_cycles < 45000)
	{ // Upper setpoint stabilisation
		_motor->FOC.Idq_req.d = _motor->meas.measure_current;
		_motor->hfi.inject = 0;
		MESCFOC(_motor);
		//      writePWM(_motor);
	}

	else if (_motor->meas.PWM_cycles < 50000)
	{ // Upper setpoint
		_motor->FOC.Idq_req.d = _motor->meas.measure_current;
		_motor->hfi.inject = 0;
		MESCFOC(_motor);
		//      writePWM(_motor);

		_motor->meas.top_V = _motor->meas.top_V + _motor->FOC.Vdq.d;
		_motor->meas.top_I = _motor->meas.top_I + _motor->FOC.Idq.d;
		_motor->meas.count_top++;
	}
	else if (_motor->meas.PWM_cycles < 50001)
	{ // Calculate R

		generateBreak(_motor);
		_motor->m.R = (_motor->meas.top_V - _motor->meas.bottom_V) / (_motor->meas.top_I - _motor->meas.bottom_I);

		// Initialise the variables for the next measurement
		// Vd_temp = _motor->FOC.Vdq.d * 1.0f;  // Store the voltage required for the high setpoint, to
		//  use as an offset for the inductance
		_motor->meas.Vq_temp = 0.0f;
		_motor->FOC.Vdq.q = 0.0f; //
		_motor->FOC.Idq_int_err.d = 0.0f;
		_motor->FOC.Idq_int_err.q = 0.0f;
		_motor->meas.count_top = 0.0f;
		_motor->meas.count_bottom = 0.0f;
		_motor->meas.top_I_L = 0.0f;
		_motor->meas.bottom_I_L = 0.0f;

		generateEnable(_motor);
	}
	/////////////////////////// Collect Ld variable//////////////////////////
	else if (_motor->meas.PWM_cycles < 80001)
	{
		// generateBreak();
		_motor->HFIType = HFI_TYPE_SPECIAL;
		_motor->hfi.inject = 1; // flag to the SVPWM writer to inject at top
		_motor->hfi.special_injectionVd = _motor->meas.measure_voltage;
		_motor->hfi.special_injectionVq = 0.0f;

		_motor->FOC.Vdq.d = _motor->meas.Vd_temp;
		_motor->FOC.Vdq.q = 0.0f;

		if (_motor->hfi.inject_high_low_now == 1)
		{
			_motor->meas.top_I_L = _motor->meas.top_I_L + _motor->FOC.Idq.d;
			_motor->meas.count_top++;
		}
		else if (_motor->hfi.inject_high_low_now == 0)
		{
			_motor->meas.bottom_I_L = _motor->meas.bottom_I_L + _motor->FOC.Idq.d;
			_motor->meas.count_bottom++;
		}
	}

	else if (_motor->meas.PWM_cycles < 80002)
	{
		generateBreak(_motor);
		_motor->m.L_D =
			fabsf((_motor->hfi.special_injectionVd) /
				  ((_motor->meas.top_I_L - _motor->meas.bottom_I_L) / (_motor->meas.count_top * _motor->FOC.pwm_period)));
		_motor->meas.top_I_Lq = 0.0f;
		_motor->meas.bottom_I_Lq = 0.0f;
		_motor->meas.count_topq = 0.0f;
		_motor->meas.count_bottomq = 0.0f;
		__NOP(); // Put a break point on it...
	}
	else if (_motor->meas.PWM_cycles < 80003)
	{
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);

		////////////////////////// Collect Lq variable//////////////////////////////
	}
	else if (_motor->meas.PWM_cycles < 100003)
	{
		//			generateBreak();
		_motor->hfi.special_injectionVd = 0.0f;
		_motor->hfi.special_injectionVq = _motor->meas.measure_voltage;
		_motor->hfi.inject = 1;					  // flag to the SVPWM writer to update at top
		_motor->FOC.Vdq.d = _motor->meas.Vd_temp; // Vd_temp to keep it aligned with D axis
		_motor->FOC.Vdq.q = 0.0f;

		if (_motor->hfi.inject_high_low_now == 1)
		{
			_motor->meas.top_I_Lq = _motor->meas.top_I_Lq + _motor->FOC.Idq.q;
			_motor->meas.count_topq++;
		}
		else if (_motor->hfi.inject_high_low_now == 0)
		{
			_motor->meas.bottom_I_Lq = _motor->meas.bottom_I_Lq + _motor->FOC.Idq.q;
			_motor->meas.count_bottomq++;
		}
	}

	else
	{
		generateBreak(_motor);
		_motor->HFIType = _motor->meas.previous_HFI_type;
		_motor->m.L_Q =
			fabsf((_motor->hfi.special_injectionVq) /
				  ((_motor->meas.top_I_Lq - _motor->meas.bottom_I_Lq) / (_motor->meas.count_top * _motor->FOC.pwm_period)));

		_motor->MotorState = MOTOR_STATE_IDLE;

		_motor->hfi.inject = 0; // flag to the SVPWM writer stop injecting at top
		_motor->hfi.special_injectionVd = 0.0f;
		_motor->hfi.special_injectionVq = 0.0f;
		_motor->hfi.Vd_injectionV = 0.0f;
		_motor->hfi.Vq_injectionV = 0.0f;
		calculateGains(_motor);
		_motor->MotorState = MOTOR_STATE_TRACKING;
		_motor->meas.PWM_cycles = 0;
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
	}
	_motor->meas.PWM_cycles++;
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

void getHallTable(MESC_motor_typedef *_motor)
{
	static int firstturn = 1;
	static int hallstate;
	hallstate = getHallState();
	static int lasthallstate = -1;
	static uint16_t pwm_count = 0;
	static int anglestep = 5; // This defines how fast the motor spins
	static uint32_t hallangles[7][2];
	static int rollover;
	hallstate = _motor->hall.current_hall_state;
	if (firstturn)
	{
		generateEnable(_motor);
		lasthallstate = hallstate;
		(void)lasthallstate;
		firstturn = 0;
	}

	////// Align the rotor////////////////////
	static uint16_t a = 65535;
	if (a) // Align time
	{
		_motor->FOC.Idq_req.d = 10.0f;
		_motor->FOC.Idq_req.q = 0.0f;

		_motor->FOC.FOCAngle = 0.0f;
		a = a - 1;
	}
	else
	{
		_motor->FOC.Idq_req.d = 10.0f;
		_motor->FOC.Idq_req.q = 0.0f;
		static int dir = 1;
		if (pwm_count < 65534)
		{
			if (_motor->FOC.FOCAngle < (anglestep))
			{
				rollover = hallstate;
			}
			if ((_motor->FOC.FOCAngle < (30000)) &&
				(_motor->FOC.FOCAngle > (29000 - anglestep)))
			{
				rollover = 0;
			}
			lasthallstate = hallstate;
			if (rollover == hallstate)
			{
				hallangles[hallstate][0] =
					hallangles[hallstate][0] +
					(uint32_t)65535; // Accumulate the angles through the sweep
			}

			_motor->FOC.FOCAngle =
				_motor->FOC.FOCAngle + anglestep; // Increment the angle
			hallangles[hallstate][0] =
				hallangles[hallstate][0] +
				_motor->FOC.FOCAngle;	// Accumulate the angles through the sweep
			hallangles[hallstate][1]++; // Accumulate the number of PWM pulses for
										// this hall state
			pwm_count = pwm_count + 1;
		}
		else if (pwm_count < 65535)
		{
			if (dir == 1)
			{
				dir = 0;
				rollover = 0;
			}
			if ((_motor->FOC.FOCAngle < (12000)) && (hallstate != _motor->hall.last_hall_state))
			{
				rollover = hallstate;
			}
			if ((_motor->FOC.FOCAngle < (65535)) &&
				(_motor->FOC.FOCAngle > (65535 - anglestep)))
			{
				rollover = 0;
			}
			lasthallstate = hallstate;
			if (rollover == hallstate)
			{
				hallangles[hallstate][0] =
					hallangles[hallstate][0] +
					(uint32_t)65535; // Accumulate the angles through the sweep
			}

			_motor->FOC.FOCAngle =
				_motor->FOC.FOCAngle - anglestep; // Increment the angle
			hallangles[hallstate][0] =
				hallangles[hallstate][0] +
				_motor->FOC.FOCAngle;	// Accumulate the angles through the sweep
			hallangles[hallstate][1]++; // Accumulate the number of PWM pulses for
										// this hall state
			pwm_count = pwm_count + 1;
		}
	}
	if (pwm_count == 65535)
	{
		generateBreak(_motor); // Debugging
		for (int i = 1; i < 7; i++)
		{
			hallangles[i][0] = hallangles[i][0] / hallangles[i][1];
			if (hallangles[i][0] > 65535)
			{
				hallangles[i][0] = hallangles[i][0] - 65535;
			}
		}
		for (int i = 0; i < 6; i++)
		{
			_motor->m.hall_table[i][2] = hallangles[i + 1][0];										  // This is the center angle of the hall state
			_motor->m.hall_table[i][3] = hallangles[i + 1][1];										  // This is the width of the hall state
			_motor->m.hall_table[i][0] = _motor->m.hall_table[i][2] - _motor->m.hall_table[i][3] / 2; // This is the start angle of the hall state
			_motor->m.hall_table[i][1] = _motor->m.hall_table[i][2] + _motor->m.hall_table[i][3] / 2; // This is the end angle of the hall state
		}
		_motor->MotorState = MOTOR_STATE_TRACKING;
		_motor->FOC.Idq_req.d = 0;
		_motor->FOC.Idq_req.q = 0;
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
	}
}

void getkV(MESC_motor_typedef *_motor)
{
	_motor->meas.previous_HFI_type = _motor->HFIType;
	_motor->HFIType = HFI_TYPE_NONE;
	_motor->hfi.inject = 0;

	static int cycles = 0;
	static HFI_type_e old_HFI_type;
	if (cycles < 2)
	{
		_motor->m.flux_linkage_max = 0.1f;
		_motor->m.flux_linkage_min = 0.00001f; // Set really wide limits
		_motor->FOC.openloop_step = 0;
		_motor->FOC.flux_observed = _motor->m.flux_linkage_min;
		old_HFI_type = _motor->HFIType;
		_motor->HFIType = HFI_TYPE_NONE;
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
	}

	flux_observer(_motor); // We run the flux observer during this

	static int count = 0;
	static uint16_t temp_angle;
	if (cycles < 60002)
	{
		_motor->FOC.Idq_req.d = _motor->meas.measure_current * 0.5f; //
		_motor->FOC.Idq_req.q = 0.0f;
		_motor->meas.angle_delta = temp_angle - _motor->FOC.FOCAngle;
		_motor->FOC.openloop_step = (uint16_t)(ERPM_MEASURE * 65536.0f / (_motor->FOC.pwm_frequency * 60.0f) * (float)cycles / 65000.0f);
		_motor->FOC.FOCAngle = temp_angle;
		OLGenerateAngle(_motor);
		temp_angle = _motor->FOC.FOCAngle;
		if (cycles == 60001)
		{
			_motor->meas.temp_flux = sqrtf(_motor->FOC.Vdq.d * _motor->FOC.Vdq.d + _motor->FOC.Vdq.q * _motor->FOC.Vdq.q) / (6.28f * (float)_motor->FOC.openloop_step * (float)_motor->FOC.pwm_frequency / 65536.0f);
			_motor->FOC.flux_observed = _motor->meas.temp_flux;
			_motor->FOC.flux_a = _motor->FOC.sincosangle.cos * _motor->FOC.flux_observed;
			_motor->FOC.flux_b = _motor->FOC.sincosangle.sin * _motor->FOC.flux_observed;
			_motor->m.flux_linkage_max = 1.7f * _motor->FOC.flux_observed;
			_motor->m.flux_linkage_min = 0.5f * _motor->FOC.flux_observed;
			_motor->meas.temp_FLA = _motor->FOC.flux_a;
			_motor->meas.temp_FLB = _motor->FOC.flux_b;
		}
		MESCFOC(_motor);
	}
	else if (cycles < 128000)
	{
		count++;
		_motor->FOC.Idq_req.d = 0.0f;
		_motor->FOC.Idq_req.q = _motor->meas.measure_closedloop_current;
		MESCFOC(_motor);
	}
	else
	{
		generateBreak(_motor);
		_motor->m.flux_linkage = _motor->FOC.flux_observed;
		calculateFlux(_motor);
		_motor->MotorState = MOTOR_STATE_TRACKING;
		_motor->HFIType = old_HFI_type;
		cycles = 0;
		_motor->HFIType = _motor->meas.previous_HFI_type;
		if (_motor->m.flux_linkage > 0.0001f && _motor->m.flux_linkage < 200.0f)
		{
			_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
		}
		else
		{
			_motor->MotorState = MOTOR_STATE_ERROR;
			generateBreak(_motor);
		}
	}
	//    writePWM(_motor);

	cycles++;
}

uint32_t tmpccmrx; // Temporary buffer which is used to turn on/off phase PWMs

// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
void phU_Break(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC1M;
	tmpccmrx &= ~TIM_CCMR1_CC1S;
	tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER &= ~TIM_CCER_CC1E;	 // disable
	htim1.Instance->CCER &= ~TIM_CCER_CC1NE; // disable
}
// Basically un-break phase U, opposite of above...
void phU_Enable(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC1M;
	tmpccmrx &= ~TIM_CCMR1_CC1S;
	tmpccmrx |= TIM_OCMODE_PWM1;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER |= TIM_CCER_CC1E;	// enable
	htim1.Instance->CCER |= TIM_CCER_CC1NE; // enable
}

void phV_Break(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC2M;
	tmpccmrx &= ~TIM_CCMR1_CC2S;
	tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER &= ~TIM_CCER_CC2E;	 // disable
	htim1.Instance->CCER &= ~TIM_CCER_CC2NE; // disable
}

void phV_Enable(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR1;
	tmpccmrx &= ~TIM_CCMR1_OC2M;
	tmpccmrx &= ~TIM_CCMR1_CC2S;
	tmpccmrx |= TIM_OCMODE_PWM1 << 8;
	htim1.Instance->CCMR1 = tmpccmrx;
	htim1.Instance->CCER |= TIM_CCER_CC2E;	// enable
	htim1.Instance->CCER |= TIM_CCER_CC2NE; // enable
}

void phW_Break(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR2;
	tmpccmrx &= ~TIM_CCMR2_OC3M;
	tmpccmrx &= ~TIM_CCMR2_CC3S;
	tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
	htim1.Instance->CCMR2 = tmpccmrx;
	htim1.Instance->CCER &= ~TIM_CCER_CC3E;	 // disable
	htim1.Instance->CCER &= ~TIM_CCER_CC3NE; // disable
}

void phW_Enable(MESC_motor_typedef *_motor)
{
	tmpccmrx = htim1.Instance->CCMR2;
	tmpccmrx &= ~TIM_CCMR2_OC3M;
	tmpccmrx &= ~TIM_CCMR2_CC3S;
	tmpccmrx |= TIM_OCMODE_PWM1;
	htim1.Instance->CCMR2 = tmpccmrx;
	htim1.Instance->CCER |= TIM_CCER_CC3E;	// enable
	htim1.Instance->CCER |= TIM_CCER_CC3NE; // enable
}

void calculateFlux(MESC_motor_typedef *_motor)
{
	_motor->m.flux_linkage_max = 1.7f * _motor->m.flux_linkage;
	_motor->m.flux_linkage_min = 0.5f * _motor->m.flux_linkage;
	_motor->m.flux_linkage_gain = 10.0f * sqrtf(_motor->m.flux_linkage);
	_motor->m.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN;
}

void calculateGains(MESC_motor_typedef *_motor)
{
	_motor->FOC.pwm_period = 1.0f / _motor->FOC.pwm_frequency;
	_motor->mtimer->Instance->ARR = HAL_RCC_GetHCLKFreq() / (((float)_motor->mtimer->Instance->PSC + 1.0f) * 2 * _motor->FOC.pwm_frequency);
	_motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR - 5; // Just short of dead center (dead center will not actually trigger the conversion)
#ifdef SINGLE_ADC
	_motor->mtimer->Instance->CCR4 = _motor->mtimer->Instance->ARR - 80; // If we only have one ADC, we need to convert early otherwise the data will not be ready in time
#endif
	_motor->FOC.PWMmid = _motor->mtimer->Instance->ARR * 0.5f;

	_motor->FOC.ADC_duty_threshold = _motor->mtimer->Instance->ARR * 0.90f;

	calculateFlux(_motor);

	_motor->FOC.Current_bandwidth = CURRENT_BANDWIDTH;
	// PID controller gains
	_motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
	_motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
	// Pole zero cancellation for series PI control
	_motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
	_motor->FOC.Iq_igain = _motor->FOC.Id_igain;

	if (_motor->FOC.FW_curr_max > input_vars.max_request_Idq.q)
	{
		_motor->FOC.FW_curr_max = 0.9f * input_vars.max_request_Idq.q; // Limit the field weakenning to 90% of the max current to avoid math errors
	}
	_motor->m.L_QD = _motor->m.L_Q - _motor->m.L_D;
	_motor->hfi.d_polarity = 1;
}

#define SVPWM_MULTIPLIER CONST_1_R_COS_30_F

void calculateVoltageGain(MESC_motor_typedef *_motor)
{
	// We need a number to convert between Va Vb and raw PWM register values
	// This number should be the bus voltage divided by the ARR register
	_motor->FOC.Vab_to_PWM =
		_motor->mtimer->Instance->ARR / _motor->Conv.Vbus;
	// We also need a number to set the maximum voltage that can be effectively
	// used by the SVPWM This is equal to
	// 0.5*Vbus*MAX_MODULATION*SVPWM_MULTIPLIER*Vd_MAX_PROPORTION
	if (_motor->ControlMode != MOTOR_CONTROL_MODE_DUTY)
	{
		_motor->FOC.Duty_scaler = 1.0f;
	}
	_motor->FOC.Vmag_max = 0.5f * _motor->Conv.Vbus *
						   MAX_MODULATION * SVPWM_MULTIPLIER * _motor->FOC.Duty_scaler;
	_motor->FOC.V_3Q_mag_max = _motor->FOC.Vmag_max * 0.75f;

	_motor->FOC.Vmag_max2 = _motor->FOC.Vmag_max * _motor->FOC.Vmag_max;
	_motor->FOC.Vd_max = 0.5f * _motor->Conv.Vbus *
						 MAX_MODULATION * SVPWM_MULTIPLIER * Vd_MAX_PROPORTION;
	_motor->FOC.Vq_max = 0.5f * _motor->Conv.Vbus *
						 MAX_MODULATION * SVPWM_MULTIPLIER * Vq_MAX_PROPORTION;

	_motor->FOC.Vdint_max = _motor->FOC.Vd_max * 0.9f; // Logic in this is to always ensure headroom for the P term
	_motor->FOC.Vqint_max = _motor->FOC.Vq_max * 0.9f;

	_motor->FOC.FW_threshold = _motor->FOC.Vmag_max * FIELD_WEAKENING_THRESHOLD;
	_motor->FOC.FW_multiplier = 1.0f / (_motor->FOC.Vmag_max * (1.0f - FIELD_WEAKENING_THRESHOLD));

	switch (_motor->HFIType)
	{ // When running HFI we want the bandwidth low, so we calculate it with each slow loop depending on whether we are HFIing or not
	case HFI_TYPE_NONE:
		__NOP();
	case HFI_TYPE_45:
		// fallthrough
	case HFI_TYPE_D:
		// fallthrough
	case HFI_TYPE_SPECIAL:

		_motor->FOC.Id_pgain = _motor->FOC.Current_bandwidth * _motor->m.L_D;
		_motor->FOC.Id_igain = _motor->m.R / _motor->m.L_D;
		// Pole zero cancellation for series PI control
		_motor->FOC.Iq_pgain = _motor->FOC.Id_pgain;
		_motor->FOC.Iq_igain = _motor->FOC.Id_igain;
		// This is the expected current magnitude we would see based on the average inductance and the injected voltage. Not particularly reliable currently.
		//_motor->FOC.HFI_Threshold = ((HFI_VOLTAGE*CONST_SQRT_2_F*2.0f)*_motor->FOC.pwm_period)/((_motor->m.L_D+_motor->m.L_Q)*0.5f);
		if (HFI_THRESHOLD == 0.0f)
		{
			_motor->hfi.HFI_toggle_voltage = _motor->Conv.Vbus * 0.05f;
			if (_motor->hfi.HFI_toggle_voltage < 3.0f)
			{
				_motor->hfi.HFI_toggle_voltage = 3.0f;
			}
		}
		else
		{
			_motor->hfi.HFI_toggle_voltage = HFI_THRESHOLD;
		}
		break;
	}
	//////Set the fault limits
	// Set the overcurrent limit according to the requested current.
	// This is important since using the board ABS_MAX may mean the motor DC resistance is high enough that a fault never trips it.
	g_hw_setup.Imax = input_vars.max_request_Idq.q * 1.5f;
	if ((g_hw_setup.Imax * 0.5f) < (0.1f * ABS_MAX_PHASE_CURRENT))
	{
		g_hw_setup.Imax = input_vars.max_request_Idq.q + 0.1f * ABS_MAX_PHASE_CURRENT;
	}
	if (g_hw_setup.Imax > ABS_MAX_PHASE_CURRENT)
	{ // Clamp the current limit to the board max
		g_hw_setup.Imax = ABS_MAX_PHASE_CURRENT;
	}
	// Set the over voltage limit dynamically, so that rapid spikes above the bus voltage are trapped.
	// This should be more convenient for working with PSUs and batteries interchangeably
	if (fabsf(_motor->FOC.Idq_req.q) < 1.0f)
	{
		g_hw_setup.Vmax = 0.995f * g_hw_setup.Vmax + 0.005f * (_motor->Conv.Vbus + 0.15f * ABS_MAX_BUS_VOLTAGE);
	}
	if (g_hw_setup.Vmax > ABS_MAX_BUS_VOLTAGE)
	{
		g_hw_setup.Vmax = ABS_MAX_BUS_VOLTAGE;
	}
}

static volatile int dp_periods = 6;
void doublePulseTest(MESC_motor_typedef *_motor)
{
	static int dp_counter;
	if (dp_counter == 0)
	{												 // Let bootstrap charge
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE); // DISABLE INTERRUPT, DANGEROUS
		phU_Enable(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter++;
	}
	else if (dp_counter <= (dp_periods - 3))
	{ // W State ON
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = htim1.Instance->ARR;
		phU_Break(_motor);
		phV_Enable(_motor);
		phW_Enable(_motor);
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter++;
	}
	else if (dp_counter == (dp_periods - 2))
	{ // Freewheel
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter++;
	}
	else if (dp_counter == (dp_periods - 1))
	{ // W short second pulse
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 200;
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter++;
	}
	else if (dp_counter == dp_periods)
	{ // Freewheel a bit to see the current
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter++;
	}
	else
	{ // Turn all off
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		test_vals.dp_current_final[dp_counter] =
			_motor->Conv.Iv;
		dp_counter = 0;
		generateBreak(_motor);
		__HAL_TIM_ENABLE_IT(_motor->mtimer, TIM_IT_UPDATE); /// RE-ENABLE INTERRUPT
		_motor->MotorState = MOTOR_STATE_TRACKING;
	}
}

uint16_t test_on_time;
uint16_t test_on_time_acc[3];
uint16_t test_counts;
void getDeadtime(MESC_motor_typedef *_motor)
{
	static int use_phase = 0;

	if (test_on_time < 1)
	{
		test_on_time = 1;
	}

	if (use_phase == 0)
	{
		htim1.Instance->CCR1 = test_on_time;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
		if (_motor->Conv.Iu < 1.0f)
		{
			test_on_time = test_on_time + 1;
		}
		if (_motor->Conv.Iu > 1.0f)
		{
			test_on_time = test_on_time - 1;
		}
		generateEnable(_motor);
		test_on_time_acc[0] = test_on_time_acc[0] + test_on_time;
	}
	if (use_phase == 1)
	{
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = test_on_time;
		htim1.Instance->CCR3 = 0;
		if (_motor->Conv.Iv < 1.0f)
		{
			test_on_time = test_on_time + 1;
		}
		if (_motor->Conv.Iv > 1.0f)
		{
			test_on_time = test_on_time - 1;
		}
		generateEnable(_motor);
		test_on_time_acc[1] = test_on_time_acc[1] + test_on_time;
	}
	if (use_phase == 2)
	{
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = test_on_time;
		if (_motor->Conv.Iw < 1.0f)
		{
			test_on_time = test_on_time + 1;
		}
		if (_motor->Conv.Iw > 1.0f)
		{
			test_on_time = test_on_time - 1;
		}
		generateEnable(_motor);
		test_on_time_acc[2] = test_on_time_acc[2] + test_on_time;
	}
	if (use_phase > 2)
	{
		generateBreak(_motor);
		_motor->MotorState = MOTOR_STATE_TRACKING;
		use_phase = 0;
		test_on_time_acc[0] = test_on_time_acc[0] >> 10;
		test_on_time_acc[1] = test_on_time_acc[1] >> 10;
		test_on_time_acc[2] = test_on_time_acc[2] >> 10;
		_motor->FOC.deadtime_comp = test_on_time_acc[0];
	}
	test_counts++;

	if (test_counts > 511)
	{
		use_phase++;
		test_counts = 0;
	}
}

void CalculateBLDCGains(MESC_motor_typedef *_motor)
{
}

// clang-format on
