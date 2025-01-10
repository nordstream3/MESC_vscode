#include "motor.h"
#include "command_line_interface.h"
#include "profile.h"
#include "motor_defaults.h"
// DANGER - Apply defaults before overriding 
#include "stm32fxxx_hal.h"
#include "math.h"
#include <stdint.h>

MOTORProfile * motor_profile = NULL;

void motor_init( MOTORProfile * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static MOTORProfile motor_profile_default;
		motor_profile_default.Imax = MAX_MOTOR_PHASE_CURRENT;
        motor_profile_default.Pmax = DEFAULT_MOTOR_POWER;
		motor_profile_default.pole_pairs =  DEFAULT_MOTOR_PP;
        motor_profile_default.L_D = DEFAULT_MOTOR_Ld;
        motor_profile_default.L_Q = DEFAULT_MOTOR_Lq;
        motor_profile_default.L_QD = DEFAULT_MOTOR_Lq-DEFAULT_MOTOR_Ld;
        motor_profile_default.R = DEFAULT_MOTOR_R;
		motor_profile_default.flux_linkage = DEFAULT_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_min = MIN_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_max = MAX_FLUX_LINKAGE;
		motor_profile_default.flux_linkage_gain = FLUX_LINKAGE_GAIN;
		motor_profile_default.non_linear_centering_gain = NON_LINEAR_CENTERING_GAIN;

        uint32_t            motor_length = sizeof(motor_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "MTR", MOTOR_PROFILE_SIGNATURE,
            &motor_profile_default, &motor_length );

        motor_profile = &motor_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "MTR FAILED" "\r" "\n" );
        }
    }
    else
    {
    	motor_profile = profile;
    }
}

void speed_motor_limiter( void )
{
	motor_profile->Pmax = 50.0f; // Watts
}
