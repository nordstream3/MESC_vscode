#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

// Forward declaration of MESC_motor_typedef
typedef struct MESC_motor_typedef MESC_motor_typedef;

typedef struct {
  float dp_current_final[10];
} MESCtest_s;

//extern MESCtest_s test_vals;


void measureResistance(MESC_motor_typedef *_motor);
void getkV(MESC_motor_typedef *_motor);
float detectHFI(MESC_motor_typedef *_motor);
void getHallTable(MESC_motor_typedef *_motor);
void calculateGains(MESC_motor_typedef *_motor);
void calculateVoltageGain(MESC_motor_typedef *_motor);
void calculateFlux(MESC_motor_typedef *_motor);
void getDeadtime(MESC_motor_typedef *_motor);

void generateBreak(MESC_motor_typedef *_motor); // Software break that does not stop the PWM timer but
                                                // disables the outputs, sum of phU,V,W_Break();
void generateEnable(MESC_motor_typedef *_motor); // Opposite of generateBreak
void phU_Break(MESC_motor_typedef *_motor);   // Turn all phase U FETs off, Tristate the ouput - For BLDC
                    // mode mainly, but also used for measuring
void phU_Enable(MESC_motor_typedef *_motor);  // Basically un-break phase U, opposite of above...
void phV_Break(MESC_motor_typedef *_motor);
void phV_Enable(MESC_motor_typedef *_motor);
void phW_Break(MESC_motor_typedef *_motor);
void phW_Enable(MESC_motor_typedef *_motor);

void CalculateBLDCGains(MESC_motor_typedef *_motor);

void doublePulseTest(MESC_motor_typedef *_motor);

#endif
