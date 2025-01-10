#include "input_vars.h"
#include "hw_setup.h"
#include "error.h"
#include "motorinstance.h"
#include "interrupt_adc.h"
#include <math.h>

input_vars_t input_vars;

void InputInit()
{

	input_vars.max_request_Idq.d = 0.0f; // Not supporting d-axis input current for now
	input_vars.min_request_Idq.d = 0.0f;
	if (!input_vars.max_request_Idq.q)
	{
		input_vars.max_request_Idq.q = MAX_IQ_REQUEST;
		input_vars.min_request_Idq.q = MIN_IQ_REQUEST; // ToDo, SETTING THESE ASSYMETRIC WILL CAUSE ISSUES WITH REVERSE..
	}

	input_vars.IC_pulse_MAX = IC_PULSE_MAX;
	input_vars.IC_pulse_MIN = IC_PULSE_MIN;
	input_vars.IC_pulse_MID = IC_PULSE_MID;
	input_vars.IC_pulse_DEADZONE = IC_PULSE_DEADZONE;
	input_vars.IC_duration_MAX = IC_DURATION_MAX;
	input_vars.IC_duration_MIN = IC_DURATION_MIN;

	if (!input_vars.adc1_MAX)
	{
		input_vars.adc1_MAX = ADC1MAX;
		input_vars.adc1_MIN = ADC1MIN;
		input_vars.adc1_OOR = ADC1OOR;
		input_vars.ADC1_polarity = ADC1_POLARITY;
	}
	if (!input_vars.adc2_MAX)
	{
		input_vars.adc2_MAX = ADC2MAX;
		input_vars.adc2_MIN = ADC2MIN;
		input_vars.adc2_OOR = ADC2OOR;
		input_vars.ADC2_polarity = ADC2_POLARITY;
	}

	input_vars.adc1_gain[0] = 1.0f / (input_vars.adc1_MAX - input_vars.adc1_MIN);
	input_vars.adc1_gain[1] = 1.0f / (input_vars.adc1_MAX - input_vars.adc1_MIN);

	input_vars.adc2_gain[0] = 1.0f / (input_vars.adc2_MAX - input_vars.adc2_MIN);
	input_vars.adc2_gain[1] = 1.0f / (input_vars.adc2_MAX - input_vars.adc2_MIN);

	// RCPWM forward gain//index [0][x] is used for Idq requests for now, might support asymmetric brake and throttle later
	input_vars.RCPWM_gain[0][0] = 1.0f / ((float)input_vars.IC_pulse_MAX - (float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE);
	input_vars.RCPWM_gain[0][1] = 1.0f / (((float)input_vars.IC_pulse_MID - (float)input_vars.IC_pulse_DEADZONE) - (float)input_vars.IC_pulse_MIN);

	if (!input_vars.input_options)
	{
		input_vars.input_options = DEFAULT_INPUT;
	}

	input_vars.UART_req = 0.0f;
	input_vars.RCPWM_req = 0.0f;
	input_vars.ADC1_req = 0.0f;
	input_vars.ADC2_req = 0.0f;
}

void collectInputs(MESC_motor_typedef *_motor)
{

	// Check if remote ADC timeouts
	if (input_vars.remote_ADC_timeout > 0)
	{
		input_vars.remote_ADC_timeout--;
	}
	else
	{
		input_vars.remote_ADC1_req = 0.0f;
		input_vars.remote_ADC2_req = 0.0f;
	}

	// Collect the requested throttle inputs

	// Remote ADC1 input
	if ((input_vars.input_options & 0b100000) && (input_vars.remote_ADC_can_id > 0))
	{
		// Do nothing. Already set
	}
	else
	{
		input_vars.remote_ADC1_req = 0.0f; // Set the input variable to zero
	}

	// Remote ADC2 input
	if ((input_vars.input_options & 0b1000000) && (input_vars.remote_ADC_can_id > 0))
	{
		// Do nothing, already set
	}
	else
	{
		input_vars.remote_ADC2_req = 0.0f; // Set the input variable to zero
	}

	// Differential ADC12 input
	if ((input_vars.input_options & 0b10000000))
	{
		// TBC, Math and logic required
		// To be filled, as signal = ext1-ext2 with error check based on ext1+ext2
	}
	else
	{
		input_vars.ADC12_diff_req = 0.0f; // Set the input variable to zero
	}

	// UART input
	if (0 == (input_vars.input_options & 0b1000))
	{
		input_vars.UART_req = 0.0f;
	}

	// RCPWM input
	if (input_vars.input_options & 0b0100)
	{
		if (input_vars.pulse_recieved)
		{
			if ((input_vars.IC_duration > input_vars.IC_duration_MIN) && (input_vars.IC_duration < input_vars.IC_duration_MAX))
			{
				if (input_vars.IC_pulse > (input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE))
				{
					input_vars.RCPWM_req = (float)(input_vars.IC_pulse - (input_vars.IC_pulse_MID + input_vars.IC_pulse_DEADZONE)) * input_vars.RCPWM_gain[0][1];
					if (fabsf(input_vars.RCPWM_req > 1.1f))
					{
						handleError(_motor, ERROR_INPUT_OOR);
					}
					if (input_vars.RCPWM_req > 1.0f)
					{
						input_vars.RCPWM_req = 1.0f;
					}
					if (input_vars.RCPWM_req < -1.0f)
					{
						input_vars.RCPWM_req = -1.0f;
					}
				}
				else if (input_vars.IC_pulse < (input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE))
				{
					input_vars.RCPWM_req = ((float)input_vars.IC_pulse - (float)(input_vars.IC_pulse_MID - input_vars.IC_pulse_DEADZONE)) * input_vars.RCPWM_gain[0][1];
					if (fabsf(input_vars.RCPWM_req > 1.1f))
					{
						handleError(_motor, ERROR_INPUT_OOR);
					}
					if (input_vars.RCPWM_req > 1.0f)
					{
						input_vars.RCPWM_req = 1.0f;
					}
					if (input_vars.RCPWM_req < -1.0f)
					{
						input_vars.RCPWM_req = -1.0f;
					}
				}
				else
				{
					input_vars.RCPWM_req = 0.0f;
				}
			}
			else
			{ // The duration of the IC was wrong; trap it and write no current request
				// Todo maybe want to implement a timeout on this, allowing spurious pulses to not wiggle the current?
				input_vars.RCPWM_req = 0.0f;
			}
		}
		else
		{ // No pulse received flag
			input_vars.RCPWM_req = 0.0f;
		}
	}
	else
	{
		input_vars.RCPWM_req = 0.0f;
	}

	// ADC2 input
	if (input_vars.input_options & 0b0010)
	{
		if (_motor->Raw.ADC_in_ext2 > input_vars.adc2_MIN)
		{
			input_vars.ADC2_req = ((float)_motor->Raw.ADC_in_ext2 - (float)input_vars.adc2_MIN) * input_vars.adc1_gain[1] * input_vars.ADC2_polarity;
			if (_motor->Raw.ADC_in_ext2 > input_vars.adc2_OOR)
			{
				// input_vars.ADC2_req = 0.0f;
				handleError(_motor, ERROR_INPUT_OOR);
			}
		}
		else
		{
			input_vars.ADC2_req = 0.0f;
		}
		if (input_vars.ADC2_req > 1.0f)
		{
			input_vars.ADC2_req = 1.0f;
		}
		if (input_vars.ADC2_req < -1.0f)
		{
			input_vars.ADC2_req = -1.0f;
		}
	}
	else
	{
		input_vars.ADC2_req = 0.0f;
	}

	// ADC1 input
	if (input_vars.input_options & 0b0001)
	{
		if (_motor->Raw.ADC_in_ext1 > input_vars.adc1_MIN)
		{
			input_vars.ADC1_req = ((float)_motor->Raw.ADC_in_ext1 - (float)input_vars.adc1_MIN) * input_vars.adc1_gain[1] * input_vars.ADC1_polarity;
			if (_motor->Raw.ADC_in_ext1 > input_vars.adc1_OOR)
			{
				// input_vars.ADC1_req = 0.0f;//If we set throttle to zero, it will immediately reset the error!
				handleError(_motor, ERROR_INPUT_OOR);
			}
		}
		else
		{
			input_vars.ADC1_req = 0.0f;
		}
		if (input_vars.ADC1_req > 1.0f)
		{
			input_vars.ADC1_req = 1.0f;
		}
		if (input_vars.ADC1_req < -1.0f)
		{
			input_vars.ADC1_req = -1.0f;
		}
	}
	else
	{
		input_vars.ADC1_req = 0.0f;
	}

#ifdef KILLSWITCH_GPIO
	if (input_vars.input_options & 0b10000)
	{ // Killswitch
		if (KILLSWITCH_GPIO->IDR & (0x01 << KILLSWITCH_IONO))
		{
			input_vars.nKillswitch = 1;
			_motor->key_bits &= ~KILLSWITCH_KEY;
		}
		else
		{
			input_vars.nKillswitch = 0;
			_motor->key_bits |= KILLSWITCH_KEY;
		}
		if (input_vars.invert_killswitch)
		{
			input_vars.nKillswitch = !input_vars.nKillswitch;
			_motor->key_bits ^= KILLSWITCH_KEY;
		}
	}
	else
	{ // If we are not using the killswitch, then it should be "on"
		input_vars.nKillswitch = 1;
		_motor->key_bits &= ~KILLSWITCH_KEY;
	}
#else
	input_vars.nKillswitch = 1;
	_motor->key_bits &= ~KILLSWITCH_KEY;
#endif
}
