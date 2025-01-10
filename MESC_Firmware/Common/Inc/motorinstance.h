#ifndef MOTORINSTANCE_H
#define MOTORINSTANCE_H

//#include <stdbool.h>
#include "stm32fxxx_hal.h"
#include "motor_control.h"
#include "motor.h"
#include "temperature.h"
//#include "MESC_BLDC.h"
#include "foc.h"
#include "hfi.h"


#ifndef SAFE_START_DEFAULT
#define SAFE_START_DEFAULT 100
#endif

#ifndef DEFAULT_ENCODER_POLARITY
#define DEFAULT_ENCODER_POLARITY 0
#endif
#ifndef ENCODER_E_OFFSET
#define ENCODER_E_OFFSET 0
#endif

#ifndef DEFAULT_SPEED_KP
#define DEFAULT_SPEED_KP 0.5f //Amps per eHz
#endif
#ifndef DEFAULT_SPEED_KI
#define DEFAULT_SPEED_KI 0.1f //Amps per eHz per slowloop period... ToDo make it per second. At 100Hz slowloop, 0.1f corresponds to a 10Hz integral.
#endif

#ifndef ADC_OFFSET_DEFAULT
#define ADC_OFFSET_DEFAULT 2048.0f
#endif

#ifndef I_MEASURE
#define I_MEASURE 20.0f //Higher setpoint for resistance measurement
#endif
#ifndef I_MEASURE_CLOSEDLOOP
#define I_MEASURE_CLOSEDLOOP 8.5f 	//After spinning up openloop and getting an approximation,
									//this current is used to driver the motor and collect a refined flux linkage
#endif
#ifndef V_MEASURE
#define V_MEASURE 4.0f 	//Voltage setpoint for measuring inductance
#endif
#ifndef ERPM_MEASURE
#define ERPM_MEASURE 3000.0f//Speed to do the flux linkage measurement at
#endif

#ifndef CURRENT_BANDWIDTH
#define CURRENT_BANDWIDTH 0.25f*PWM_FREQUENCY //Note, current bandwidth in rads-1, PWMfrequency in Hz, so the default is about fPWM = 25xcurrent bandwidth.
#endif

#ifndef MAX_MODULATION
#define MAX_MODULATION 0.95f //default is 0.95f, can allow higher or lower. up to
							//1.1 stable with 5 sector switching,
							//1.05 is advised as max for low side shunts
#endif

#define Vd_MAX_PROPORTION 0.3f //These are only used when hard clamping limits are enabled, not when SQRT circle limitation used
#define Vq_MAX_PROPORTION 0.95f

//Position and speed estimator defaults
#ifndef PLL_KP
#define PLL_KP 0.5f
#endif
#ifndef PLL_KI
#define PLL_KI 0.02f
#endif

#ifndef POS_KP
#define POS_KP 0.0002f
#endif
#ifndef POS_KI
#define POS_KI 0.1f
#endif
#ifndef POS_KD
#define POS_KD 0.002f
#endif

// Debug
#define DWT_CYCCNT ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES *DWT_CYCCNT

#ifndef HALL_IIR
#define HALL_IIR 0.05f
#endif

#define HALL_IIRN (1.0f-HALL_IIR)


typedef struct {
	int Iu;
	int Iv;
	int Iw;

	int Vbus;

	int Vu;
	int Vv;
	int Vw;

	int MOSu_T;
	int MOSv_T;
	int MOSw_T;

	TEMP MOS_temp;

	int Motor_T;
	TEMP Motor_temp;

	int16_t ADC_in_ext1;
	int16_t ADC_in_ext2;
} MESC_raw_typedef;

typedef struct {
	float Iu;
	float Iv;
	float Iw;
} MESC_offset_typedef;

typedef struct {
	float Iu;
	float Iv;
	float Iw;

	float Vbus;

	float Vu;
	float Vv;
	float Vw;

	float MOSu_T;
	float MOSv_T;
	float MOSw_T;

	float Motor_T;
} MESC_Converted_typedef;

typedef struct {
	//Measure resistance
	float top_V;
	float bottom_V;
	float top_I;
	float bottom_I;
	float count_top;
	float count_topq;
	float count_bottom;
	float count_bottomq;

	float Vd_temp;
	float Vq_temp;
	float top_I_L;
	float bottom_I_L;
	float top_I_Lq;
	float bottom_I_Lq;
	int PWM_cycles;
	HFI_type_e previous_HFI_type;

	//getkV
	int angle_delta;
	float temp_flux;
	float temp_FLA;
	float temp_FLB;

	float hfi_voltage;

	float measure_current;
	float measure_voltage;
	float measure_closedloop_current;
} MESCmeas_s;

typedef struct {
	float dir;
	int current_hall_state;
	uint16_t current_hall_angle;
	int last_hall_state;
	uint16_t last_hall_angle;
	float ticks_since_last_observer_change;
	float last_observer_period;
	float one_on_last_observer_period;
	float angular_velocity;
	float angle_step;

	int hall_error;
} MESChall_s;

typedef struct{
	uint16_t OL_periods;
	uint16_t OL_countdown;
	int  closed_loop;
	int sector;
	int direction;
	float PWM_period;

	float I_set;
	float I_meas;
	float V_meas;
	float V_meas_sect[6];
	float rising_int;
	float falling_int;
	float rising_int_st;
	float falling_int_st;
	float last_p_error;

	float I_error;
	float int_I_error;
	float I_pgain;
	float I_igain;

	float com_flux;
	float flux_integral;
	float last_flux_integral;

	float V_bldc;
	float V_bldc_to_PWM;
	uint16_t BLDC_PWM;

} MESCBLDC_s;


/////////////Position controller data
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float error;
	uint32_t last_pll_pos;
	float d_pos;
	float p_error;
	float d_error;
	float int_error;
	uint32_t set_position;
	int32_t deadzone;
} MESCPos_s;

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////Main typedef for starting a motor instance////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
typedef struct MESC_motor_typedef {
	TIM_HandleTypeDef *mtimer; //3 phase PWM timer
	TIM_HandleTypeDef *stimer; //Timer that services the slowloop
	TIM_HandleTypeDef *enctimer; //Timer devoted to taking incremental encoder inputs
//problematic if there is no SPI allocated//	SPI_HandleTypeDef *encspi; //The SPI we have configured to talk to the encoder for this motor instance
	motor_state_e MotorState;
	motor_sensor_mode_e MotorSensorMode;
	motor_startup_sensor_e SLStartupSensor;
	motor_control_mode_e ControlMode;
	motor_control_type_e MotorControlType;
	HighPhase_e HighPhase;
	HFI_type_e HFIType;
	MESC_raw_typedef Raw;
	MESC_Converted_typedef Conv;
	MESC_offset_typedef offset;

	// Anonymous union inside structure
	//union { Base_foc_s FOC; MESCfoc_s FOC_; };
	foc_s FOC;

	hfi_s hfi;
	MESCPos_s pos;
	MESCBLDC_s BLDC;
	MOTORProfile m;
	MESCmeas_s meas;
	MESChall_s hall;
	bool conf_is_valid;
	int32_t safe_start[2];
	uint32_t key_bits; //When any of these are low, we keep the motor disabled
	bool sample_now;
	bool sample_no_auto_send;
} MESC_motor_typedef;


extern MESC_motor_typedef mtr[NUM_MOTORS];


void MESCInit(MESC_motor_typedef *_motor);
void initialiseInverter(MESC_motor_typedef *_motor);
void generateBreakAll();	//Disables all drives

#endif
