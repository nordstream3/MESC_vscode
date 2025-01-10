#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>

#define MOTOR_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','M','P','E')

struct MOTORProfile
{
    float       Imax;         // Amp
    float       Vmax;         // Volt
    float       Pmax;         // Watt
    uint32_t    RPMmax;       // 1/minute
    uint8_t     pole_pairs;
    uint8_t     direction;
    uint8_t     _[2];
    float       L_D;          // Henry
    float       L_Q;          // Henry
    float 		L_QD;		  // Henry
    float       R;            // Ohm
    float       flux_linkage; // Weber
    float       flux_linkage_min;
    float       flux_linkage_max;
    float       flux_linkage_gain;
    float       non_linear_centering_gain; //Weber/second
    float 		hall_flux[6][2]; //Weber
    uint16_t 	hall_table[6][4];  // Lookup table, populated by the getHallTable()
    uint16_t 	enc_counts;
};

typedef struct MOTORProfile MOTORProfile;

extern MOTORProfile * motor_profile;

void motor_init( MOTORProfile * const profile );

#endif
