#ifndef SPEED_H
#define SPEED_H

#include <inttypes.h>

#define SPEED_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','S','P','E')

#define NUM_HALL_STATES (UINT32_C(6))

struct HallEntry
{
    uint16_t min;
    uint16_t max;
};

typedef struct HallEntry HallEntry;

struct SPEEDProfile
{
    struct
    {
    uint16_t    encoder_offset;
    HallEntry   hall_states[NUM_HALL_STATES];
    }           sensor;

    struct
    {
    uint32_t    motor;      // Teeth on motor
    uint32_t    wheel;      // Teeth on wheel
    }           gear_ratio; // Use 1:1 for no gear ratio

    struct
    {
    float       diameter;   // In wheel size units (inches/centimetres)
    float       conversion; // Conversion from wheel size units to speedometer units (miles/kilometres per hour)
    }           wheel;

};

typedef struct SPEEDProfile SPEEDProfile;

extern SPEEDProfile const * speed_profile;

void speed_init( SPEEDProfile const * const profile );

void speed_register_vars( float const * const eHz, uint8_t const * const pp );

float speed_get( void );

void speed_motor_limiter( void );

#endif
