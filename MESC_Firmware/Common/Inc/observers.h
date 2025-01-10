#ifndef OBSERVERS_H
#define OBSERVERS_H

#include "motorinstance.h"

void flux_observer(MESC_motor_typedef *_motor);
void flux_observer_V2(MESC_motor_typedef *_motor);
void angleObserver(MESC_motor_typedef *_motor);
void hallAngleEstimator();  // Going to attempt to make a similar hall angle
                            // estimator that rolls the hall state into the main
                            // function, and calls a vector table to find the
                            // angle from hall offsets.
void LRObserver(MESC_motor_typedef *_motor);
void LRObserverCollect(MESC_motor_typedef *_motor);
void HallFluxMonitor(MESC_motor_typedef *_motor);

#endif
