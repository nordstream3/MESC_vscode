#ifndef ERROR_H_
#define ERROR_H_

//Includes
#include "stm32fxxx_hal.h"


// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;

//Variables
extern  uint32_t MESC_errors;
extern  uint32_t MESC_all_errors;

struct MESC_log_vars
{
	float current_A;
	float current_B;
	float current_C;
	float voltage;
	float motor_flux;
	float flux_a;
	float flux_b;
	int count;
};



//Define anticipated errors possible, not all will be implemented
#define ERROR_OVERCURRENT_PHA 1
#define ERROR_OVERCURRENT_PHB 2
#define ERROR_OVERCURRENT_PHC 3
#define ERROR_OVERVOLTAGE 4
#define ERROR_UNDERVOLTAGE 5
#define ERROR_BRK 6
#define ERROR_OVERTEMPU 7
#define ERROR_OVERTEMPV 8
#define ERROR_OVERTEMPW 9
#define ERROR_OVERTEMP_MOTOR 10
#define ERROR_HARDFAULT 11
#define ERROR_BUSFAULT 12
#define ERROR_NMI 13
#define ERROR_MEMFAULT 14
#define ERROR_USAGE 15
#define ERROR_ADC_OUT_OF_RANGE_IA 16
#define ERROR_ADC_OUT_OF_RANGE_IB 17
#define ERROR_ADC_OUT_OF_RANGE_IC 18
#define ERROR_ADC_OUT_OF_RANGE_VBUS 19
#define ERROR_WDG 20
#define ERROR_UNBALANCED_CURRENT 21
#define ERROR_MEASUREMENT_FAIL 22
#define ERROR_DETECTION_FAIL 23
#define ERROR_HALL0 24
#define ERROR_HALL7 25
#define ERROR_MATH 26
#define ERROR_INPUT_OOR 27
#define ERROR_STARTUP 28

void handleError(MESC_motor_typedef *_motor, uint32_t error_code);
void clearErrors();

#endif /* INC_MESCERROR_H_ */
