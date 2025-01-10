#include "motor_control.h"
#include "motorinstance.h"
#include "input_vars.h"
#include "logging.h"
#include "error.h"
#include "hw_setup.h"
#include "interrupt_adc.h"
#include "observers.h"
#include "encoder.h"
#include "measurements.h"
#include "bldc.h"
#include "position.h"
#include "stm32fxxx_hal.h"
#include <math.h>
#include <stdlib.h>


motor_state_e MotorState;
motor_sensor_mode_e MotorSensorMode;
motor_error_type_e MotorError;
motor_direction_e MotorDirection;
motor_control_type_e MotorControlType;
test_mode_e TestMode;


//static void SlowStartup(MESC_motor_typedef *_motor);
static void calculatePower(MESC_motor_typedef *_motor);
static void LimitFWCurrent(MESC_motor_typedef *_motor);
static void houseKeeping(MESC_motor_typedef *_motor);
static void clampBatteryPower(MESC_motor_typedef *_motor);
static void ThrottleTemperature(MESC_motor_typedef *_motor);
static void FWRampDown(MESC_motor_typedef *_motor);
static int isHandbrake();
static void safeStart(MESC_motor_typedef *_motor);
static void VICheck(MESC_motor_typedef *_motor);
static void RunSpeedControl(MESC_motor_typedef *_motor);

float Square(float x) { return ((x) * (x)); }


// The fastloop runs at PWM timer counter top, which is when the new ADC current
// readings arrive.
// The first few clock cycles of the interrupt should not use the adc readings,
// since the currents require approximately 1us = 144 clock cycles (f405) and 72
// clock cycles (f303) to convert.
int16_t diff;
void fastLoop(MESC_motor_typedef *_motor)
{
	uint32_t cycles = CPU_CYCLES;
	// Call this directly from the TIM top IRQ
	_motor->hall.current_hall_state = getHallState(); // ToDo, this macro is not applicable to dual motors
	// First thing we ever want to do is convert the ADC values
	// to real, useable numbers.
	ADCConversion(_motor);

	switch (_motor->MotorState)
	{

	case MOTOR_STATE_INITIALISING:
		initialiseInverter(_motor);
		break;

	case MOTOR_STATE_RUN:
		switch (_motor->MotorSensorMode)
		{
		case MOTOR_SENSOR_MODE_SENSORLESS:
#ifdef USE_HALL_START
			if (_motor->FOC.hall_start_now)
			{
				_motor->FOC.flux_a = HALL_IIRN * _motor->FOC.flux_a + HALL_IIR * _motor->m.hall_flux[_motor->hall.current_hall_state - 1][0];
				_motor->FOC.flux_b = HALL_IIRN * _motor->FOC.flux_b + HALL_IIR * _motor->m.hall_flux[_motor->hall.current_hall_state - 1][1];
				if (fabsf(_motor->FOC.Vdq.q - _motor->m.R * _motor->FOC.Idq_smoothed.q) > HALL_VOLTAGE_THRESHOLD)
				{
					flux_observer(_motor); // For some reason, this does not seem to work well at stationary;
					// it results in vibrations at standstill, although it smooths the transition. Therefore, start it a bit later.
				}
				else
				{
					_motor->FOC.FOCAngle = (uint16_t)(32768.0f + 10430.0f * fast_atan2(_motor->FOC.flux_b, _motor->FOC.flux_a)) - 32768;
				}
			}
			else if (_motor->FOC.enc_start_now)
			{
				_motor->FOC.flux_a = 0.95f * _motor->FOC.flux_a + _motor->FOC.enccos * 0.05f * _motor->m.flux_linkage;
				_motor->FOC.flux_b = 0.95f * _motor->FOC.flux_b + _motor->FOC.encsin * 0.05f * _motor->m.flux_linkage;
				flux_observer(_motor);
			}
			else
			{
				flux_observer(_motor);
			}
#else
			flux_observer(_motor);
#endif
			MESCFOC(_motor);
			//			writePWM(_motor);
			break;
		case MOTOR_SENSOR_MODE_HALL:
			_motor->hfi.inject = 0;
			hallAngleEstimator();
			angleObserver(_motor);
			MESCFOC(_motor);
			//			writePWM(_motor);
			break;
		case MOTOR_SENSOR_MODE_OPENLOOP:
			getIncEncAngle(_motor); // Add this for setting up encoder
			OLGenerateAngle(_motor);
			MESCFOC(_motor);
			//			writePWM(_motor);
			break;
		case MOTOR_SENSOR_MODE_ABSOLUTE_ENCODER:
			_motor->FOC.enc_period_count++;
			_motor->FOC.FOCAngle = _motor->FOC.enc_angle + (uint16_t)((float)(_motor->FOC.enc_period_count) * (float)_motor->FOC.enc_pwm_step);
			MESCFOC(_motor);
			//			writePWM(_motor);
			break;
		case MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER:
			getIncEncAngle(_motor);
			_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
			MESCFOC(_motor);
			//			writePWM(_motor);
			break;
		case MOTOR_SENSOR_MODE_HFI:
			break;
		} // End of MotorSensorMode switch
		break;

	case MOTOR_STATE_TRACKING:
#ifdef HAS_PHASE_SENSORS
		// Track using BEMF from phase sensors
		generateBreak(_motor);
		getRawADCVph(_motor);
		ADCPhaseConversion(_motor);
		MESCTrack(_motor);
		if (_motor->MotorSensorMode == MOTOR_SENSOR_MODE_HALL)
		{
			hallAngleEstimator(_motor);
			angleObserver(_motor);
		}
		else if (_motor->MotorSensorMode == MOTOR_SENSOR_MODE_SENSORLESS)
		{
			flux_observer(_motor);
#ifdef USE_HALL_START
			HallFluxMonitor(_motor);
#endif
		}
		else if ((_motor->MotorSensorMode == MOTOR_SENSOR_MODE_ABSOLUTE_ENCODER))
		{
			_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		}
		else if ((_motor->MotorSensorMode == MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER))
		{
			getIncEncAngle(_motor);
			_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
		}
#endif

		break;

	case MOTOR_STATE_OPEN_LOOP_STARTUP:
		// Same as open loop
		_motor->FOC.openloop_step = 60;
		OLGenerateAngle(_motor);
		MESCFOC(_motor);
		//    	writePWM(_motor);
		// Write the PWM values
		break;

	case MOTOR_STATE_OPEN_LOOP_TRANSITION:
		// Run open loop
		// Run observer
		// RunFOC
		// Weighted average of the outputs N PWM cycles
		// Write the PWM values
		break;

	case MOTOR_STATE_IDLE:
		generateBreak(_motor);
		// Do basically nothing
		break;

	case MOTOR_STATE_DETECTING:

		if ((_motor->hall.current_hall_state == 7))
		{ // no hall sensors detected, all GPIO pulled high
			_motor->MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
			_motor->MotorState = MOTOR_STATE_GET_KV;
		}
		else if (_motor->hall.current_hall_state == 0)
		{
			_motor->MotorState = MOTOR_STATE_ERROR;
			MotorError = MOTOR_ERROR_HALL0;
		}
		else
		{
			// hall sensors detected
			_motor->MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
			getHallTable(_motor);
			MESCFOC(_motor);
			//        writePWM(_motor);
		}
		break;

	case MOTOR_STATE_MEASURING:
		// Every PWM cycle we enter this function until
		// the resistance measurement has converged at a
		// good value. Once the measurement is complete,
		// Rphase is set, and this is no longer called
		measureResistance(_motor);
		break;

	case MOTOR_STATE_GET_KV:
		getkV(_motor);

		break;

	case MOTOR_STATE_ERROR:
		generateBreak(_motor); // Generate a break state (software disabling all PWM)
							   // Now panic and freak out
		break;

	case MOTOR_STATE_ALIGN:
		// Turn on at a given voltage at electricalangle0;
		break;

	case MOTOR_STATE_TEST:
		if (TestMode == TEST_TYPE_DOUBLE_PULSE)
		{
			// Double pulse test
			doublePulseTest(_motor);
		}
		else if (TestMode == TEST_TYPE_DEAD_TIME_IDENT)
		{
			// Here we are going to pull all phases low, and then increase the duty on one phase until we register a current response.
			// This duty represents the dead time during which there is no current response
			getDeadtime(_motor);
		}
		else if (TestMode == TEST_TYPE_HARDWARE_VERIFICATION)
		{
			// Here we want a function that pulls all phases low, then all high and verifies a response
			// Then we want to show a current response with increasing phase duty
		}
		break;

	case MOTOR_STATE_RECOVERING:
		deadshort(_motor); // Function to startup motor from running without phase sensors
		break;

	case MOTOR_STATE_SLAMBRAKE:
		if ((fabsf(_motor->Conv.Iu) > input_vars.max_request_Idq.q) ||
			(fabsf(_motor->Conv.Iv) > input_vars.max_request_Idq.q) ||
			(fabsf(_motor->Conv.Iw) > input_vars.max_request_Idq.q))
		{
			generateBreak(_motor);
		}
		else
		{
			generateEnable(_motor);
			//    	  htim1.Instance->CCR1 = 0;
			//    	  htim1.Instance->CCR2 = 0;
			//    	  htim1.Instance->CCR3 = 0;
			// We use "0", since this corresponds to all high side FETs off, always, and all low side ones on, always.
			// This means that current measurement can continue on low side and phase shunts, so over current protection remains active.
			if (_motor->MotorSensorMode == MOTOR_SENSOR_MODE_INCREMENTAL_ENCODER)
			{
				getIncEncAngle(_motor);
				_motor->FOC.FOCAngle = _motor->FOC.enc_angle;
				//     		  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>16384){
				//     			  if((_motor->FOC.parkangle-_motor->FOC.FOCAngle)>32768){
				//    			  _motor->FOC.parkangle = _motor->FOC.FOCAngle+16384;
				//     			  }
				//    		  }
				//     		  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)>16384){
				//    			  if((_motor->FOC.FOCAngle-_motor->FOC.parkangle)<32767){
				//    				  _motor->FOC.parkangle = _motor->FOC.FOCAngle-16384;
				//    			  }
				//    		  }
				diff = (int)(_motor->FOC.FOCAngle - _motor->FOC.parkangle);
				if (abs(diff) > 16384)
				{
					if (diff < 0)
					{
						_motor->FOC.parkangle = _motor->FOC.FOCAngle + 16000;
						__NOP();
					}
					else
					{
						_motor->FOC.parkangle = _motor->FOC.FOCAngle - 16000;
						__NOP();
					}
				}
				if (abs(diff) < 8000)
				{
					_motor->FOC.Vdq.q = 0.0f;
					_motor->FOC.Vdq.d = 0.0f;
					_motor->FOC.park_current_now = 0.0f;
				}
				else
				{
					_motor->FOC.Idq_req.q = -_motor->FOC.park_current * (float)diff / (float)8192; // Fill with some PID logic
					_motor->FOC.Idq_req.d = 0.0f;												   //
					if (diff > 0)
					{
						_motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q + _motor->FOC.park_current;
					}
					else
					{
						_motor->FOC.Idq_req.q = _motor->FOC.Idq_req.q - _motor->FOC.park_current;
					}
					MESCFOC(_motor);
					_motor->FOC.park_current_now = _motor->FOC.Idq_req.q;
				}
			}
			else
			{
				_motor->FOC.Vdq.q = 0.0f;
				_motor->FOC.Vdq.d = 0.0f;
				_motor->FOC.park_current_now = 0.0f;
			}
		}
		break;
	case MOTOR_STATE_RUN_BLDC:
		getRawADCVph(_motor);
		ADCPhaseConversion(_motor);
		BLDCCommute(_motor);
		__NOP();
		break;

	default:
		_motor->MotorState = MOTOR_STATE_ERROR;
		generateBreak(_motor);
		break;
	}
#ifdef SOFTWARE_ADC_REGULAR
	HAL_ADC_Start(&hadc1); // Try to eliminate the HAL call, slow and inefficient. Leaving this here for now.
						   // hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
#endif

#ifdef USE_LR_OBSERVER
	LRObserverCollect();
#endif
#ifdef USE_SPI_ENCODER
	tle5012(_motor);
#endif

	// RunPLL for all angle options
	_motor->FOC.PLL_angle = _motor->FOC.PLL_angle + (int16_t)_motor->FOC.PLL_int + (int16_t)_motor->FOC.PLL_error;
	_motor->FOC.PLL_error = _motor->FOC.PLL_kp * (int16_t)(_motor->FOC.FOCAngle - (_motor->FOC.PLL_angle & 0xFFFF));
	_motor->FOC.PLL_int = _motor->FOC.PLL_int + _motor->FOC.PLL_ki * _motor->FOC.PLL_error;
	_motor->FOC.eHz = _motor->FOC.PLL_int * _motor->FOC.pwm_frequency * 0.00001526f; // 1/65536

#ifdef LOGGING
	if (lognow)
	{
		static int post_error_samples;
		if (_motor->MotorState != MOTOR_STATE_ERROR && _motor->sample_now == false)
		{
			logVars(_motor);
			post_error_samples = POST_ERROR_SAMPLES;
		}
		else
		{ // If we have an error state, we want to keep the data surrounding the error log, including some sampled during and after the fault
			if (post_error_samples > 1)
			{
				logVars(_motor);
				post_error_samples--;
			}
			else if (post_error_samples == 1)
			{
				print_samples_now = 1;
				//_motor->sample_now = false;
				post_error_samples--;
			}
			else
			{
				__NOP();
			}
		}
	}
#endif
	_motor->FOC.cycles_fastloop = CPU_CYCLES - cycles;
}

void slowLoop(MESC_motor_typedef *_motor)
{
	// In this loop, we will fetch the throttle values, and run functions that
	// are critical, but do not need to be executed very often e.g. adjustment
	// for battery voltage change
	/// Process buttons for direction
	// if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == 0){
	//	input_vars.ADC1_polarity = -1.0f;
	// }
	// if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0){
	//	input_vars.ADC1_polarity = 1.0f;
	// }
	houseKeeping(_motor);  // General dross that keeps things ticking over, like nudging the observer
	collectInputs(_motor); // Get all the throttle inputs

	switch (_motor->ControlMode)
	{
	case MOTOR_CONTROL_MODE_TORQUE:
		// We just scale and sum the input current requests
		_motor->FOC.Idq_prereq.q = input_vars.UART_req +
								   input_vars.max_request_Idq.q * (input_vars.ADC1_req + input_vars.ADC2_req +
																   input_vars.RCPWM_req + input_vars.ADC12_diff_req +
																   input_vars.remote_ADC1_req + input_vars.remote_ADC2_req);

		// Clamp the Q component; d component is not directly requested
		if (_motor->FOC.Idq_prereq.q > input_vars.max_request_Idq.q)
		{
			_motor->FOC.Idq_prereq.q = input_vars.max_request_Idq.q;
		}
		if (_motor->FOC.Idq_prereq.q < input_vars.min_request_Idq.q)
		{
			_motor->FOC.Idq_prereq.q = input_vars.min_request_Idq.q;
		}
		break;
	case MOTOR_CONTROL_MODE_POSITION:
		RunPosControl(_motor);
		break;
	case MOTOR_CONTROL_MODE_SPEED:
		// TBC PID loop to convert eHz feedback to an iq request
		RunSpeedControl(_motor);
		break;
	case MOTOR_CONTROL_MODE_DUTY:
		_motor->FOC.Idq_prereq = input_vars.max_request_Idq;
		// Sum the total duty request
		float total_in = input_vars.ADC1_req + input_vars.ADC2_req +
						 input_vars.RCPWM_req + input_vars.UART_req + input_vars.ADC12_diff_req +
						 input_vars.remote_ADC1_req + input_vars.remote_ADC2_req;
		if (fabsf(total_in) > 0.01f)
		{
			if (total_in > 1.0f)
			{
				total_in = 1.0f;
			}
			if (total_in < -1.0f)
			{
				total_in = -1.0f;
			}
			_motor->FOC.Duty_scaler = fabsf(total_in); // Assign the duty here
		}
		else
		{
			total_in = 0.001f;
			_motor->FOC.Duty_scaler = fabsf(total_in);
		}
		break;
	case MOTOR_CONTROL_MODE_MEASURING:
		_motor->MotorSensorMode = MOTOR_SENSOR_MODE_OPENLOOP;
		_motor->HFIType = HFI_TYPE_NONE;
		_motor->FOC.Id_pgain = 0.0f;
		_motor->FOC.Iq_pgain = 0.0f;
		_motor->FOC.Id_igain = 0.0f;
		_motor->FOC.Iq_igain = 0.0f;
		_motor->FOC.openloop_step = (uint16_t)(600.0f * 65536 / _motor->FOC.pwm_frequency); // 300Hz tone
		_motor->FOC.Idq_int_err.d = 10.0f;													// 1V
		_motor->FOC.Idq_int_err.q = 0.0f;													// 1V
		_motor->FOC.Current_bandwidth = 0.0f;
		_motor->FOC.PLL_int = 0.0f;
		_motor->FOC.PLL_ki = 0.0f;
		_motor->FOC.PLL_ki = 0.0f;
		_motor->FOC.PLL_error = 0.0f;

		_motor->m.R = 10.0f * _motor->FOC.Idq_smoothed.d / (_motor->FOC.Idq_smoothed.d * _motor->FOC.Idq_smoothed.d + _motor->FOC.Idq_smoothed.q * _motor->FOC.Idq_smoothed.q);
		_motor->m.L_D = -10.0f * _motor->FOC.Idq_smoothed.q / (2.0f * 3.1415f * 600.0f * (_motor->FOC.Idq_smoothed.d * _motor->FOC.Idq_smoothed.d + _motor->FOC.Idq_smoothed.q * _motor->FOC.Idq_smoothed.q));
		if (_motor->MotorState != MOTOR_STATE_ERROR)
		{
			_motor->MotorState = MOTOR_STATE_RUN;
		}
		break;
	case MOTOR_CONTROL_MODE_HANDBRAKE:
		if ((_motor->MotorState == MOTOR_STATE_RUN) || (_motor->MotorState == MOTOR_STATE_TRACKING))
		{
			if ((fabsf(_motor->FOC.Vdq.q) < 0.1f * _motor->Conv.Vbus))
			{ // Check it is not error or spinning fast!
				_motor->MotorState = MOTOR_STATE_SLAMBRAKE;
			}
			else
			{ // We are going fast, just disable PWM
				_motor->MotorState = MOTOR_STATE_TRACKING;
				generateBreak(_motor);
			}
		}
		float req_now = (input_vars.UART_req + input_vars.max_request_Idq.q * (input_vars.ADC1_req + input_vars.ADC2_req + input_vars.RCPWM_req));

		_motor->FOC.Idq_prereq.q = req_now;
		if ((req_now > (0.05f * input_vars.max_request_Idq.q)) && (req_now > _motor->FOC.park_current_now) && (_motor->MotorState == MOTOR_STATE_SLAMBRAKE))
		{
			_motor->MotorState = MOTOR_STATE_TRACKING;
			_motor->ControlMode = MOTOR_CONTROL_MODE_TORQUE;
		}
		break;
	default:
		__NOP();
		break;
	}
	/////////////////Handle the safe startup
	safeStart(_motor);
	/////////////////Handle the keybits (initialised flag, killswitch and safestart)
	if ((_motor->key_bits))
	{
		_motor->FOC.Idq_prereq.q = 0.0f;
		_motor->FOC.Idq_prereq.d = 0.0f;
	}
	///////////////////////Run the state machine//////////////////////////////////
	switch (_motor->MotorState)
	{
	case MOTOR_STATE_TRACKING:
		ThrottleTemperature(_motor);
		_motor->hfi.was_last_tracking = 1;
		// Seperate based on control mode. We NEED to have a fallthrough here in transition state!
		// Does not seem possible to use nested switches due to fallthrough requirement :(
		if (_motor->ControlMode == MOTOR_CONTROL_MODE_TORQUE)
		{
			if (isHandbrake())
			{
				_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;
			}
			if (fabsf(_motor->FOC.Idq_prereq.q) > 0.2f)
			{
#ifdef HAS_PHASE_SENSORS
				if (_motor->MotorControlType == MOTOR_CONTROL_TYPE_FOC)
				{
					_motor->MotorState = MOTOR_STATE_RUN;
				}
				else if (_motor->MotorControlType == MOTOR_CONTROL_TYPE_BLDC)
				{
					_motor->MotorState = MOTOR_STATE_RUN_BLDC;
				}
#else
				_motor->MotorState = MOTOR_STATE_RECOVERING;
				break;
#endif
				// fallthrough to RUN, no break!
			}
			else
			{
				// Remain in tracking
				break;
			}
		}
		else if (_motor->ControlMode == MOTOR_CONTROL_MODE_POSITION)
		{
			if (_motor->MotorState != MOTOR_STATE_ERROR)
			{
				_motor->MotorState = MOTOR_STATE_RUN;
			}
		}
		else if (_motor->ControlMode == MOTOR_CONTROL_MODE_SPEED)
		{
			if (_motor->FOC.speed_req > 10.0f)
			{
				_motor->MotorState = MOTOR_STATE_RUN;
				// fallthrough to RUN, no break!
			}
			else
			{
				break;
			}
		}
		else if (_motor->ControlMode == MOTOR_CONTROL_MODE_DUTY)
		{
			if (_motor->FOC.Duty_scaler > 0.01f)
			{
				_motor->MotorState = MOTOR_STATE_RUN;
				// fallthrough to RUN, no break!
			}
			else
			{
				break;
			}
		}
		// end of ControlMode switch

	case MOTOR_STATE_RUN:
		calculatePower(_motor);
		ThrottleTemperature(_motor); // Gradually ramp down the Q current if motor or FETs are getting hot
		RunMTPA(_motor);			 // Process MTPA
		LimitFWCurrent(_motor);		 // Process FW -> Iq reduction
		clampBatteryPower(_motor);	 // Prevent too much power being drawn from the battery
#ifdef USE_LR_OBSERVER
		LRObserver();
#endif
		// Assign the Idqreq to the PI input
		_motor->FOC.Idq_req.q = _motor->FOC.Idq_prereq.q;
		_motor->FOC.Idq_req.d = _motor->FOC.Idq_prereq.d;
		if (input_vars.UART_dreq)
		{
			_motor->FOC.Idq_req.d = input_vars.UART_dreq;
		} // Override the calcs if a specific d is requested
		generateEnable(_motor);
		switch (_motor->ControlMode)
		{
		case MOTOR_CONTROL_MODE_TORQUE:
			if (((fabsf(_motor->FOC.Idq_prereq.q) < 0.1f)))
			{ // Request current small, FW not active
				if ((_motor->FOC.FW_current > -0.5f))
				{
					_motor->MotorState = MOTOR_STATE_TRACKING;
					generateBreak(_motor);
				}
				else
				{
					FWRampDown(_motor);
				}
			}
			if (isHandbrake())
			{
				_motor->ControlMode = MOTOR_CONTROL_MODE_HANDBRAKE;
			}
			break;

		case MOTOR_CONTROL_MODE_SPEED:
			if (fabsf(_motor->FOC.speed_req) < 10.0f)
			{
				_motor->MotorState = MOTOR_STATE_TRACKING;
				generateBreak(_motor);
			}
			break;
		case MOTOR_CONTROL_MODE_DUTY:
			if (_motor->FOC.Duty_scaler > 0.01f)
			{
			}
			else
			{
				_motor->MotorState = MOTOR_STATE_TRACKING;
				generateBreak(_motor);
			}
			break;
		case MOTOR_CONTROL_MODE_POSITION:
			__NOP();
		default:
			break;
		} // end of ControlMode switch

		SlowStartup(_motor);
		break;
	case MOTOR_STATE_RUN_BLDC:
		// Assign the Idqreq to the PI input
		_motor->BLDC.I_set = _motor->FOC.Idq_prereq.q;
		break;

	case MOTOR_STATE_ERROR:
		// add recovery stuff
		switch (_motor->ControlMode)
		{
		case MOTOR_CONTROL_MODE_HANDBRAKE:
			// fallthrough
		case MOTOR_CONTROL_MODE_TORQUE:
			if (fabsf(_motor->FOC.Idq_prereq.q) < 0.1f)
			{
				_motor->MotorState = MOTOR_STATE_TRACKING;
				VICheck(_motor); // Immediately return it to error state if there is still a critical fault condition active
				clearErrors();
			}
			break;
		case MOTOR_CONTROL_MODE_SPEED:
			if (fabsf(_motor->FOC.speed_req) < 0.1f)
			{
				_motor->MotorState = MOTOR_STATE_TRACKING;
				VICheck(_motor); // Immediately return it to error state if there is still a critical fault condition active
			}
			break;
		case MOTOR_CONTROL_MODE_DUTY:
			if (fabsf(_motor->FOC.Duty_scaler) < 0.01f)
			{
				_motor->MotorState = MOTOR_STATE_TRACKING;
				VICheck(_motor); // Immediately return it to error state if there is still a critical fault condition active
			}
		default:
			break;
		}
		break;
	case MOTOR_STATE_SLAMBRAKE:
		__NOP();
		// We might want to do something if there is a handbrake state? Like exiting this state?
		break;
	default:
		__NOP();
		// This accounts for all the initialising, test, measuring... procedures.
		// We basically just want to do nothing and let them get on with their job.
		break;
	}
	/////////////////End of Switch state machine///////////////////////////////

	calculateVoltageGain(_motor);
}




void houseKeeping(MESC_motor_typedef *_motor)
{
	////// Unpuc the observer kludge
	// The observer gets into a bit of a state if it gets close to
	// flux linked = 0 for both accumulators, the angle rapidly changes
	// as it oscillates around zero. Solution... just kludge it back out.
	// This only happens at stationary when it is useless anyway.
	if ((_motor->FOC.flux_a * _motor->FOC.flux_a + _motor->FOC.flux_b * _motor->FOC.flux_b) <
		0.25f * _motor->FOC.flux_observed * _motor->FOC.flux_observed)
	{
		_motor->FOC.flux_a = 2.5f * _motor->FOC.flux_a; //_motor->FOC.flux_observed;
		_motor->FOC.flux_b = 2.5f * _motor->FOC.flux_b; //_motor->FOC.flux_observed;
														// This was altered because otherwise basing the flux on the observed flux
		// causes issues a step change in direction, so at low speed - e.g. during hall sensor startup - it causes instability.
	}

	// Speed tracker
	if (abs(_motor->FOC.PLL_int) > 10000.0f)
	{
		// The PLL has run away locking on to aliases; 10000 implies 6.5 pwm periods per sin wave, which is ~3000eHz, 180kerpm at 20kHz PWM frequency.
		// While it IS possible to run faster than this, it is not a sensible use case and will not be supported.
		_motor->FOC.PLL_int = 0;
	}
	// Translate the eHz to eRPM
	if (_motor->m.pole_pairs > 0)
	{ // avoid divide by zero
		_motor->FOC.mechRPM = _motor->FOC.eHz * 60.0f / (float)(_motor->m.pole_pairs);
	}
	// Shut down if we are burning the hall sensors //Legacy code, can probably be removed...
	//	if(getHallState()==0){//This happens when the hall sensors overheat it seems.
	//	  	  if (MotorError == MOTOR_ERROR_NONE) {
	//	  		    speed_motor_limiter();
	//	  	  }
	//	  	  MotorError = MOTOR_ERROR_HALL0;
	//	    }else /*if(getHallState()==7){
	//	  	  MotorError = MOTOR_ERROR_HALL7;
	//	    } else */{
	//	  	  if (MotorError != MOTOR_ERROR_NONE) {
	//	  		  // TODO speed_road();
	//	  	  }
	//	  	  MotorError = MOTOR_ERROR_NONE;
	//	    }
}

void calculatePower(MESC_motor_typedef *_motor)
{
	////// Calculate the current power
	_motor->FOC.currentPower.d = 1.5f * (_motor->FOC.Vdq.d * _motor->FOC.Idq_smoothed.d);
	_motor->FOC.currentPower.q = 1.5f * (_motor->FOC.Vdq.q * _motor->FOC.Idq_smoothed.q);
	_motor->FOC.Ibus = (_motor->FOC.currentPower.d + _motor->FOC.currentPower.q) / _motor->Conv.Vbus;
}

void LimitFWCurrent(MESC_motor_typedef *_motor)
{
	// Account for Field weakening current
	// MTPA is already conservative of the current limits
	float mag = (Square(_motor->FOC.Idq_prereq.q) + Square(_motor->FOC.FW_current));
	if (mag > Square(input_vars.max_request_Idq.q))
	{
		float Iqmax2 = Square(input_vars.max_request_Idq.q) - Square(_motor->FOC.FW_current);
		if (Iqmax2 > 0)
		{ // Avoid hardfault
			if (_motor->FOC.Idq_prereq.q > 0)
			{
				_motor->FOC.Idq_prereq.q = sqrtf(Iqmax2);
			}
			else
			{
				_motor->FOC.Idq_prereq.q = -sqrtf(Iqmax2);
			}
		}
		else
		{ // Negative result, FW larger than allowable current
			_motor->MotorState = MOTOR_STATE_ERROR;
			handleError(_motor, ERROR_MATH);
			_motor->FOC.FW_current = 0.0f;
		}
	}
}

void clampBatteryPower(MESC_motor_typedef *_motor)
{
	/////// Clamp the max power taken from the battery
	_motor->FOC.reqPower = 1.5f * fabsf(_motor->FOC.Vdq.q * _motor->FOC.Idq_prereq.q);
	if (_motor->FOC.reqPower > _motor->m.Pmax)
	{
		if (_motor->FOC.Idq_prereq.q > 0.0f)
		{
			_motor->FOC.Idq_prereq.q = _motor->m.Pmax / (fabsf(_motor->FOC.Vdq.q) * 1.5f);
		}
		else
		{
			_motor->FOC.Idq_prereq.q = -_motor->m.Pmax / (fabsf(_motor->FOC.Vdq.q) * 1.5f);
		}
	}
}

void FWRampDown(MESC_motor_typedef *_motor)
{
	// Ramp down the field weakening current
	// Do NOT assign motorState here, since it could override error states
	if (_motor->FOC.Vdq.q < 0.0f)
	{
		_motor->FOC.Idq_req.q = 0.2f; // Apply a brake current
	}
	if (_motor->FOC.Vdq.q > 0.0f)
	{
		_motor->FOC.Idq_req.q = -0.2f; // Apply a brake current
	}
}

static void handleThrottleTemperature(MESC_motor_typedef *_motor, float const T, float *const dTmax, int const errorcode)
{
	float dT = 0.0f;
	TEMPState const temp_state = temp_check(&_motor->Raw.Motor_temp, T, &dT);
#define TMAX(a, b) (((a) > (b)) ? (a) : (b))
	*dTmax = TMAX(*dTmax, dT);
#undef TMAX
	if (temp_state == TEMP_STATE_OVERHEATED)
	{
		handleError(_motor, errorcode);
	}
}

float dTmax = 0.0f;
void ThrottleTemperature(MESC_motor_typedef *_motor)
{
	dTmax = 0.0f;

	_motor->Conv.MOSu_T = 0.99f * _motor->Conv.MOSu_T + 0.01f * temp_read(&_motor->Raw.MOS_temp, _motor->Raw.MOSu_T);
	_motor->Conv.MOSv_T = 0.99f * _motor->Conv.MOSv_T + 0.01f * temp_read(&_motor->Raw.MOS_temp, _motor->Raw.MOSv_T);
	_motor->Conv.MOSw_T = 0.99f * _motor->Conv.MOSw_T + 0.01f * temp_read(&_motor->Raw.MOS_temp, _motor->Raw.MOSw_T);
	_motor->Conv.Motor_T = 0.99f * _motor->Conv.Motor_T + 0.01f * temp_read(&_motor->Raw.Motor_temp, _motor->Raw.Motor_T);

	handleThrottleTemperature(_motor, _motor->Conv.MOSu_T, &dTmax, ERROR_OVERTEMPU);
	handleThrottleTemperature(_motor, _motor->Conv.MOSv_T, &dTmax, ERROR_OVERTEMPV);
	handleThrottleTemperature(_motor, _motor->Conv.MOSw_T, &dTmax, ERROR_OVERTEMPW);
	handleThrottleTemperature(_motor, _motor->Conv.Motor_T, &dTmax, ERROR_OVERTEMP_MOTOR);

	_motor->FOC.T_rollback = (1.0f - dTmax / (_motor->Raw.MOS_temp.limit.Tmax - _motor->Raw.MOS_temp.limit.Thot));
	if (_motor->FOC.T_rollback <= 0.0f)
	{
		_motor->FOC.T_rollback = 0.0f;
	}
	if (_motor->FOC.T_rollback > 1.0f)
	{
		_motor->FOC.T_rollback = 1.0f;
	}
	if (_motor->FOC.Idq_prereq.q > (_motor->FOC.T_rollback * input_vars.max_request_Idq.q))
	{
		_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * input_vars.max_request_Idq.q;
	}
	if (_motor->FOC.Idq_prereq.q < (_motor->FOC.T_rollback * input_vars.min_request_Idq.q))
	{
		_motor->FOC.Idq_prereq.q = _motor->FOC.T_rollback * input_vars.min_request_Idq.q;
	}
}

int handbrakenow;
int isHandbrake()
{
#ifdef HANDBRAKE_GPIO
	handbrakenow = HANDBRAKE_GPIO->IDR & (0x01 << HANDBRAKE_IONO);
	if (HANDBRAKE_GPIO->IDR & (0x01 << HANDBRAKE_IONO))
	{
		return 1;
	}
	else
	{
		return 0;
	}
#else
	return 0;
#endif
}

void safeStart(MESC_motor_typedef *_motor)
{
	if ((_motor->FOC.Idq_req.q == 0.0f) && (_motor->FOC.Idq_prereq.q == 0.0f))
	{
		_motor->safe_start[1]++;
	}
	else if (_motor->safe_start[1] < _motor->safe_start[0])
	{
		_motor->safe_start[1] = 0;
	}
	if (_motor->safe_start[1] >= _motor->safe_start[0])
	{
		_motor->safe_start[1] = _motor->safe_start[0];
		_motor->key_bits &= ~SAFESTART_KEY;
	}
}

void VICheck(MESC_motor_typedef *_motor)
{ // Check currents, voltages are within panic limits

	if (_motor->Raw.Iu > g_hw_setup.RawCurrLim)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHA);
	}
	if (_motor->Raw.Iv > g_hw_setup.RawCurrLim)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHB);
	}
	if (_motor->Raw.Iw > g_hw_setup.RawCurrLim)
	{
		handleError(_motor, ERROR_OVERCURRENT_PHC);
	}
	if (_motor->Raw.Vbus > g_hw_setup.RawVoltLim)
	{
		handleError(_motor, ERROR_OVERVOLTAGE);
	}
}

// Speed controller
void RunSpeedControl(MESC_motor_typedef *_motor)
{
	float speed_error;
	if (_motor->MotorState == MOTOR_STATE_RUN)
	{

		speed_error = _motor->FOC.speed_kp * (_motor->FOC.speed_req - _motor->FOC.eHz);
		// Bound the proportional term before we do anything with it
		// We use the symetric terms here to allow fast PID ramps
		if (speed_error > input_vars.max_request_Idq.q)
		{
			speed_error = input_vars.max_request_Idq.q;
		}
		if (speed_error < -input_vars.max_request_Idq.q)
		{
			speed_error = -input_vars.max_request_Idq.q;
		}

		_motor->FOC.speed_error_int = _motor->FOC.speed_error_int + speed_error * _motor->FOC.speed_ki;
		// Bound the integral term...
		// Again, using symmetric terms
		if (_motor->FOC.speed_error_int > input_vars.max_request_Idq.q)
		{
			_motor->FOC.speed_error_int = input_vars.max_request_Idq.q;
		}
		if (_motor->FOC.speed_error_int < -input_vars.max_request_Idq.q)
		{
			_motor->FOC.speed_error_int = -input_vars.max_request_Idq.q;
		}

		// Apply the PID
		_motor->FOC.Idq_prereq.q = _motor->FOC.speed_error_int + speed_error;
		// Bound the overall...
		// Now we use asymmetric terms to stop it regenerating too much
		if (_motor->FOC.Idq_prereq.q > input_vars.max_request_Idq.q)
		{
			_motor->FOC.Idq_prereq.q = input_vars.max_request_Idq.q;
		}
		if (_motor->FOC.Idq_prereq.q < input_vars.min_request_Idq.q)
		{
			_motor->FOC.Idq_prereq.q = input_vars.min_request_Idq.q;
		}
	}
	else
	{
		// Set zero
		_motor->FOC.Idq_prereq.q = 0.0f;
		_motor->FOC.speed_error_int = 0.0f;
	}
}
