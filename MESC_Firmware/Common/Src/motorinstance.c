
#include "motorinstance.h"
#include "hw_setup.h"
#include "error.h"
#include "logging.h"
#include "interrupt_adc.h"
#include "measurements.h"
#include "input_vars.h"
#include "conversions.h"

// Debug
#define DEMCR_TRCENA 0x01000000
#define DEMCR (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA (1 << 0)


MESC_motor_typedef mtr[NUM_MOTORS];

extern TIM_HandleTypeDef htim1;


void MESCInit(MESC_motor_typedef *_motor)
{
	// Initialize deadtime compensation value
	// This is used inside function writePWM() of foc.h, when macro DEADTIME_COMP is defined
	_motor->FOC.deadtime_comp = DEADTIME_COMP_V;

#ifdef STM32L4 // For some reason, ST have decided to have a different name for the L4 timer DBG freeze...
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
#else
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
#endif
#ifdef FASTLED
	FASTLED->MODER |= 0x1 << (FASTLEDIONO * 2);
	FASTLED->MODER &= ~(0x2 << (FASTLEDIONO * 2));
#endif
#ifdef SLOWLED
	SLOWLED->MODER |= 0x1 << (SLOWLEDIONO * 2);
	SLOWLED->MODER &= ~(0x2 << (SLOWLEDIONO * 2));
#endif

#ifdef KILLSWITCH_GPIO
	KILLSWITCH_GPIO->MODER &= ~(0b11 << (2 * KILLSWITCH_IONO));
#endif

#ifdef HANDBRAKE_GPIO
	HANDBRAKE_GPIO->MODER &= ~(0b11 << (2 * HANDBRAKE_IONO));
#endif

#ifdef BRAKE_DIGITAL_GPIO
	BRAKE_DIGITAL_GPIO->MODER &= ~(0b11 << (2 * BRAKE_DIGITAL_IONO));
#endif

#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->MODER |= 0x1 << (INV_ENABLE_M1_IONO * 2);
	INV_ENABLE_M1->MODER &= ~(0x2 << (INV_ENABLE_M1_IONO * 2));
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->MODER |= 0x1 << (INV_ENABLE_M2_IONO * 2);
	INV_ENABLE_M2->MODER &= ~(0x2 << (INV_ENABLE_M2_IONO * 2));
#endif
	_motor->safe_start[0] = SAFE_START_DEFAULT;

	_motor->MotorState = MOTOR_STATE_IDLE;

	// enable cycle counter
	DEMCR |= DEMCR_TRCENA;
	DWT_CTRL |= CYCCNTENA;
	_motor->m = *motor_profile;

	_motor->offset.Iu = ADC_OFFSET_DEFAULT;
	_motor->offset.Iv = ADC_OFFSET_DEFAULT;
	_motor->offset.Iw = ADC_OFFSET_DEFAULT;

	_motor->MotorState = MOTOR_STATE_INITIALISING;

	// At this stage, we initialise the options
	_motor->MotorControlType = MOTOR_CONTROL_TYPE_FOC;
	_motor->ControlMode = DEFAULT_CONTROL_MODE;

	_motor->MotorSensorMode = DEFAULT_SENSOR_MODE;
	_motor->HFIType = DEFAULT_HFI_TYPE;

	_motor->meas.measure_current = I_MEASURE;
	_motor->meas.measure_voltage = V_MEASURE;
	_motor->meas.measure_closedloop_current = I_MEASURE_CLOSEDLOOP;
	_motor->FOC.pwm_frequency = PWM_FREQUENCY;
	_motor->meas.hfi_voltage = HFI_VOLTAGE;

	// Init Hall sensor
	_motor->hall.dir = 1.0f;
	_motor->hall.ticks_since_last_observer_change = 65535.0f;
	_motor->hall.last_observer_period = 65536.0f;
	_motor->hall.one_on_last_observer_period = 1.0f;
	_motor->hall.angular_velocity = 0.0f;
	_motor->hall.angle_step = 0.0f;

	// PWM Encoder
	_motor->FOC.enc_offset = ENCODER_E_OFFSET;
	_motor->FOC.encoder_polarity_invert = DEFAULT_ENCODER_POLARITY;
	_motor->FOC.enc_period_count = 1; // Avoid /0s

	// ABI Incremental encoder
	_motor->m.enc_counts = 4096; // Default to this, common for many motors. Avoid div0.
	_motor->FOC.enc_ratio = 65536 / _motor->m.enc_counts;

	_motor->hall.hall_error = 0;
	// Init the BLDC
	_motor->BLDC.com_flux = _motor->m.flux_linkage * 1.65f; // 0.02f;
	_motor->BLDC.direction = -1;

	// Init the speed controller
	_motor->FOC.speed_kp = DEFAULT_SPEED_KP; // 0.01 = 10A/1000eHz
	_motor->FOC.speed_ki = DEFAULT_SPEED_KI; // Trickier to set since we want this to be proportional to the ramp speed? Not intuitive? Try 0.1; ramp in 1/10 of a second @100Hz.
	// Init the Duty controller
	_motor->FOC.Duty_scaler = 1.0f; // We want this to be 1.0f for everything except duty control mode.
	// Init the PLL values
	_motor->FOC.PLL_kp = PLL_KP;
	_motor->FOC.PLL_ki = PLL_KI;
	//	//Init the POS values
	_motor->pos.Kp = POS_KP;
	_motor->pos.Ki = POS_KI;
	_motor->pos.Kd = POS_KD;
	//
	_motor->Raw.MOS_temp.V = 3.3f;
	_motor->Raw.MOS_temp.R_F = MESC_TEMP_MOS_R_F;
	_motor->Raw.MOS_temp.adc_range = 4096;
	_motor->Raw.MOS_temp.method = MESC_TEMP_MOS_METHOD;
	_motor->Raw.MOS_temp.schema = MESC_TEMP_MOS_SCHEMA;
	_motor->Raw.MOS_temp.parameters.SH.Beta = MESC_TEMP_MOS_SH_BETA;
	_motor->Raw.MOS_temp.parameters.SH.r = MESC_TEMP_MOS_SH_R;
	_motor->Raw.MOS_temp.parameters.SH.T0 = CVT_CELSIUS_TO_KELVIN_F(25.0f);
	_motor->Raw.MOS_temp.parameters.SH.R0 = MESC_TEMP_MOS_SH_R0;
	_motor->Raw.MOS_temp.limit.Tmin = CVT_CELSIUS_TO_KELVIN_F(-15.0f);
	_motor->Raw.MOS_temp.limit.Thot = CVT_CELSIUS_TO_KELVIN_F(80.0f);
	_motor->Raw.MOS_temp.limit.Tmax = CVT_CELSIUS_TO_KELVIN_F(100.0f);

	_motor->Raw.Motor_temp.V = 3.3f;
	_motor->Raw.Motor_temp.R_F = MESC_TEMP_MOTOR_R_F;
	_motor->Raw.Motor_temp.adc_range = 4096;
	_motor->Raw.Motor_temp.method = MESC_TEMP_MOTOR_METHOD;
	_motor->Raw.Motor_temp.schema = MESC_TEMP_MOTOR_SCHEMA;
	_motor->Raw.Motor_temp.parameters.SH.Beta = MESC_TEMP_MOTOR_SH_BETA;
	_motor->Raw.Motor_temp.parameters.SH.r = MESC_TEMP_MOTOR_SH_R;
	_motor->Raw.Motor_temp.parameters.SH.T0 = CVT_CELSIUS_TO_KELVIN_F(25.0f);
	_motor->Raw.Motor_temp.parameters.SH.R0 = MESC_TEMP_MOTOR_SH_R0;
	_motor->Raw.Motor_temp.limit.Tmin = CVT_CELSIUS_TO_KELVIN_F(-15.0f);
	_motor->Raw.Motor_temp.limit.Thot = CVT_CELSIUS_TO_KELVIN_F(80.0f);
	_motor->Raw.Motor_temp.limit.Tmax = CVT_CELSIUS_TO_KELVIN_F(100.0f);

	// Init the FW
	_motor->FOC.FW_curr_max = FIELD_WEAKENING_CURRENT; // test number, to be stored in user settings

	mesc_init_1(_motor);

	HAL_Delay(1000); // Give the everything else time to start up (e.g. throttle,
					 // controller, PWM source...)

	mesc_init_2(_motor);

	hw_init(_motor); // Populate the resistances, gains etc of the PCB - edit within
					 // this function if compiling for other PCBs
// Reconfigure dead times
// This is only useful up to 1500ns for 168MHz clock, 3us for an 84MHz clock
#ifdef CUSTOM_DEADTIME
	uint32_t tempDT;
	uint32_t tmpbdtr = 0U;
	tmpbdtr = mtr->mtimer->Instance->BDTR;
	tempDT = (uint32_t)(((float)CUSTOM_DEADTIME * (float)HAL_RCC_GetHCLKFreq()) / (float)1000000000.0f);
	if (tempDT < 128)
	{
		MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
	}
	else
	{
		uint32_t deadtime = CUSTOM_DEADTIME;
		deadtime = deadtime - (uint32_t)(127.0f * 1000000000.0f / (float)HAL_RCC_GetHCLKFreq());
		tempDT = 0b10000000 + (uint32_t)(((float)deadtime * (float)HAL_RCC_GetHCLKFreq()) / (float)2000000000.0f);
		MODIFY_REG(tmpbdtr, TIM_BDTR_DTG, tempDT);
	}
	mtr->mtimer->Instance->BDTR = tmpbdtr;
#endif

	// Start the PWM channels, reset the counter to zero each time to avoid
	// triggering the ADC, which in turn triggers the ISR routine and wrecks the
	// startup
	mesc_init_3(_motor);
	// Set the keybits
	_motor->key_bits = UNINITIALISED_KEY + KILLSWITCH_KEY + SAFESTART_KEY;

	while (_motor->MotorState == MOTOR_STATE_INITIALISING)
	{
		// At this point, the ADCs have started and we want nothing to happen until initialisation complete
		generateBreakAll();
	}
	calculateGains(_motor);
	calculateVoltageGain(_motor);

#ifdef LOGGING
	lognow = 1;
#endif

#ifdef USE_SPI_ENCODER
	_motor->FOC.enc_offset = ENCODER_E_OFFSET;
#endif
	//  __HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);

	// Start the slowloop timer
	HAL_TIM_Base_Start(_motor->stimer);
	// Here we can auto set the prescaler to get the us input regardless of the main clock
	__HAL_TIM_SET_PRESCALER(_motor->stimer, ((HAL_RCC_GetHCLKFreq()) / 1000000 - 1));
	__HAL_TIM_SET_AUTORELOAD(_motor->stimer, (1000000 / SLOWTIM_SCALER) / SLOW_LOOP_FREQUENCY); // Run slowloop at 100Hz
	__HAL_TIM_ENABLE_IT(_motor->stimer, TIM_IT_UPDATE);
	InputInit();

	// htim1.Instance->BDTR |=TIM_BDTR_MOE;
	//  initialising the comparators triggers the break state,
	//  so turn it back on
	//  At this point we just let the whole thing run off into interrupt land, and
	//  the fastLoop() starts to be triggered by the ADC conversion complete
	//  interrupt

	_motor->conf_is_valid = true;

	// Lock it in initialising while the offsets not completed
	//	while(_motor->key_bits & UNINITIALISED_KEY){
	//		_motor->MotorState = MOTOR_STATE_INITIALISING;
	//		HAL_Delay(0);
	//		generateBreakAll();
	//	}
}

void initialiseInverter(MESC_motor_typedef *_motor)
{
	static int Iuoff, Ivoff, Iwoff;
	Iuoff += (float)_motor->Raw.Iu;
	Ivoff += (float)_motor->Raw.Iv;
	Iwoff += (float)_motor->Raw.Iw;

	static int initcycles = 0;
	initcycles = initcycles + 1;
	// Exit the initialisation after 1000cycles
	if (initcycles == 1000)
	{
		calculateGains(_motor);
		calculateVoltageGain(_motor);
		_motor->FOC.flux_b = 0.001f;
		_motor->FOC.flux_a = 0.001f;

		_motor->offset.Iu = Iuoff / initcycles;
		_motor->offset.Iv = Ivoff / initcycles;
		_motor->offset.Iw = Iwoff / initcycles;
		initcycles = 0;
		Iuoff = 0;
		Ivoff = 0;
		Iwoff = 0;
		if ((_motor->offset.Iu > 1500) && (_motor->offset.Iu < 2600) && (_motor->offset.Iv > 1500) && (_motor->offset.Iv < 2600) && (_motor->offset.Iw > 1500) && (_motor->offset.Iw < 2600))
		{
			// ToDo, do we want some safety checks here like offsets being roughly correct?
			_motor->MotorState = MOTOR_STATE_TRACKING;
			_motor->key_bits &= ~UNINITIALISED_KEY;
			htim1.Instance->BDTR |= TIM_BDTR_MOE;
		}
		else
		{
			handleError(_motor, ERROR_STARTUP);
			// Should just loop until this succeeds
		}
	}
}

void generateBreakAll()
{
#ifdef INV_ENABLE_M1
	INV_ENABLE_M1->BSRR = INV_ENABLE_M1_IO << 16U; // Write the inverter enable pin low
#endif
#ifdef INV_ENABLE_M2
	INV_ENABLE_M2->BSRR = INV_ENABLE_M2_IO << 16U; // Write the inverter enable pin low
#endif
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		generateBreak(&mtr[i]);
	}
}
