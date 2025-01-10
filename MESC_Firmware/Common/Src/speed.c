#include "speed.h"
#include "motor.h"
#include "profile.h"
#include "command_line_interface.h"
#include "conversions.h"
#include <stdint.h>

SPEEDProfile const * speed_profile = NULL;

static float rev_speed; // Speedometer units per motor revolution

static float   const * speed_eHz  = NULL; // Motor revolutions
static uint8_t const * speed_pp   = NULL; // pole pairs

void speed_init( SPEEDProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static SPEEDProfile speed_profile_default =
		{
        	{0,{{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}}},
			{1,1},
			{26.0f,CONST_INCHES_PER_MILE_F}
		};
        uint32_t            speed_length = sizeof(speed_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "SPD", SPEED_PROFILE_SIGNATURE,
            &speed_profile_default, &speed_length );

        speed_profile = &speed_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "SPD FAILED" "\r" "\n" );
        }
    }
    else
    {
    	speed_profile = profile;
    }

    float const gear_ratio          = (float)speed_profile->gear_ratio.motor
                                    / (float)speed_profile->gear_ratio.wheel;
    float const wheel_circumference = (speed_profile->wheel.diameter * CONST_PI_F);

    rev_speed = (gear_ratio * wheel_circumference * CONST_SECONDS_PER_HOUR_F)
              / speed_profile->wheel.conversion;
}

void speed_register_vars( float const * const eHz, uint8_t const * const pp )
{
    speed_eHz = eHz;
    speed_pp  = pp;
}

float speed_get( void )
{
    return ((*speed_eHz * rev_speed) / *speed_pp);
}
