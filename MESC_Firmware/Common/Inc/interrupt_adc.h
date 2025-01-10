#ifndef INTERRUPT_ADC_H
#define INTERRUPT_ADC_H

#include "hw_setup.h"

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;


//Default options which can be overwritten by user
#ifndef PWM_FREQUENCY
#define PWM_FREQUENCY 20000 //This is half the VESC zero vector frequency; i.e. 20k is equivalent to VESC 40k
#endif

#ifndef SLOW_LOOP_FREQUENCY
#define SLOW_LOOP_FREQUENCY 100 //Frequency of the slow loop (MIN: 16Hz!)
#endif

#ifndef SLOWTIM_SCALER
#define SLOWTIM_SCALER 1 //There is an annoying /2 on the htim2 and other random timers that is present in the F405 but not the F401 and some others. Unclear where to get this from HAL library.
#endif

#ifndef DEADTIME_COMP_V
#define DEADTIME_COMP_V 0 	//Arbitrary value for starting, needs determining through TEST_TYP_DEAD_TIME_IDENT.
#endif						//Basically this is half the time between MOSoff and MOSon
							//and needs dtermining experimentally, either with openloop
							//sin wave drawing or by finding the zero current switching "power knee point"
							//Not defining this uses 5 sector and overmodulation compensation
							//5 sector is harder on the low side FETs (for now)but offers equal performance at low speed, better at high speed.
#ifndef OVERMOD_DT_COMP_THRESHOLD
#define OVERMOD_DT_COMP_THRESHOLD 100	//Prototype concept that allows 100% (possibly greater) modulation by
										//skipping turn off when the modulation is close to VBus, then compensating next cycle.
										//Only works with 5 sector (bottom clamp) - comment out #define SEVEN_SECTOR
#endif


#ifndef MIN_HALL_FLUX_VOLTS
#define MIN_HALL_FLUX_VOLTS 10.0f
#endif


#ifndef MIN_IQ_REQUEST
#define MIN_IQ_REQUEST -0.1f
#endif

#ifndef DEADSHORT_CURRENT
#define DEADSHORT_CURRENT 30.0f
#endif




#ifndef DEFAULT_CONTROL_MODE
#define DEFAULT_CONTROL_MODE MOTOR_CONTROL_MODE_TORQUE
#endif

#ifndef ABS_MIN_BUS_VOLTAGE
#define ABS_MIN_BUS_VOLTAGE 12.0f //We do not run below the typical gate driver safe working voltage.
#endif


#ifndef ADC1OOR
#define ADC1OOR 4095
#endif

#ifndef ADC2OOR
#define ADC2OOR 4095
#endif

/*enum MESCADC
{
    ADCIU,
    ADCIV,
    ADCIW,
};

enum FOCChannels
{
    FOC_CHANNEL_PHASE_I,
    FOC_CHANNEL_DC_V,
    FOC_CHANNEL_PHASE_V,

    FOC_CHANNELS
};

enum RCPWMMode{
	THROTTLE_ONLY,
	THROTTLE_REVERSE,
	THROTTLE_NO_REVERSE
};*/


void MESC_PWM_IRQ_handler(MESC_motor_typedef *_motor);
							//Put this into the PWM interrupt,
void MESC_ADC_IRQ_handler(MESC_motor_typedef *_motor);
							//Put this into the ADC interrupt
							//Alternatively, the PWM and ADC IRQ handlers can be
							//stacked in a single interrupt occurring once per period
							//but HFI will be lost
void ADCConversion(MESC_motor_typedef *_motor);  // Roll this into the V_I_Check? less branching, can
                       // probably reduce no.ops and needs doing every cycle
                       // anyway...
// convert currents from uint_16 ADC readings into float A and uint_16 voltages
// into float volts Since the observer needs the Clark transformed current, do
// the Clark and Park transform now
void ADCPhaseConversion(MESC_motor_typedef *_motor);

void MESC_Slow_IRQ_handler(MESC_motor_typedef *_motor); 	//This loop should run off a slow timer e.g. timer 3,4... at 20-50Hz in reset mode
														//Default setup is to use a 50Hz RCPWM input, which if the RCPWM is not present will run at 20Hz
														//If entered from update (reset, CC1) no data available for the PWM in. If entered from CC2, new PWM data available
void MESC_IC_Init(
#ifdef IC_TIMER
TIM_HandleTypeDef _IC_TIMER
#endif
);

void MESC_IC_IRQ_Handler(MESC_motor_typedef *_motor, uint32_t SR, uint32_t CCR1, uint32_t CCR2);

#endif
