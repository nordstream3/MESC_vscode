#include "battery.h"
#include "command_line_interface.h"
#include "profile.h"

#include <stddef.h>


static BATProfile const * bat_profile = NULL;

static float grad_upper;    // Amp Hour per Volt
static float grad_lower;    // Amp Hour per Volt
static float Cscale;        // Charge scale factor (Cmax..Clow)

void bat_init( BATProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        static BATProfile bat_profile_default;
        uint32_t          bat_length = sizeof(bat_profile_default);

        ProfileStatus const ret = profile_get_entry(
            "BAT", BAT_PROFILE_SIGNATURE,
            &bat_profile_default, &bat_length );

        bat_profile = &bat_profile_default;

        if (ret != PROFILE_STATUS_SUCCESS)
        {
            cli_reply( "BAT FAILED" "\r" "\n" );
        }
    }
    else
    {
    	bat_profile = profile;
    }

	bat_notify_profile_update();

	float const u_dV = bat_profile->cell.Vmax - bat_profile->cell.Vmid;
	float const u_dC = bat_profile->cell.Cmax - bat_profile->cell.Cmid;

	grad_upper = u_dC / u_dV;

	float const l_dV = bat_profile->cell.Vmid - bat_profile->cell.Vmin;
	float const l_dC = bat_profile->cell.Cmid;

	grad_lower = l_dC / l_dV;
}

void bat_notify_profile_update( void )
{
    switch (bat_profile->display)
    {
        case BAT_DISPLAY_PERCENT:
            Cscale = (100.0f / (bat_profile->cell.Cmax - bat_profile->cell.Clow));
            break;
        case BAT_DISPLAY_AMPHOUR:
            Cscale = (float)bat_profile->battery.parallel;
            break;
    }
}

float battery_get_power(
    float const Iq, float const Vq,
    float const Id, float const Vd )
{
    return ((Iq * Vq) + (Id * Vd));
}

float battery_get_current(
    float const Iq, float const Vq,
    float const Id, float const Vd,
    float const Vbat )
{
    float const power = battery_get_power( Iq, Vq, Id, Vd );
    return (power / Vbat);
}

float bat_get_charge_level( float const V, float const I)
{
    float const Vbat = (V + (I * bat_profile->battery.ESR));
    float const Vcell = (Vbat / (float)bat_profile->battery.series);
    float dV;
    float  C;

    if (Vcell <= bat_profile->cell.Vlow)
    {
        C = 0;
    }
    else if (Vcell >= bat_profile->cell.Vmax)
    {
        C = (bat_profile->cell.Cmax - bat_profile->cell.Clow);
    }
    else
    {
        if (Vcell > bat_profile->cell.Vmid)
        {
            dV = (Vcell - bat_profile->cell.Vmid);
            C  = bat_profile->cell.Cmid + (grad_upper * dV);
        }
        else
        {
            dV = (Vcell - bat_profile->cell.Vmin);
            C  = (grad_lower * dV);
        }

        if (bat_profile->cell.Clow >= C)
        {
            C = 0;
        }
        else
        {
            C = (C - bat_profile->cell.Clow);
        }
    }

    C = (C * Cscale);

    return C;
}

float bat_get_level_voltage( float const L_C )
{
    float const Lrem = bat_profile->cell.Clow + (L_C / Cscale);
    float V;

    if (Lrem <= bat_profile->cell.Clow)
    {
        V = bat_profile->cell.Vlow;
    }
    else if (Lrem <= bat_profile->cell.Cmid)
    {
        float const dL = (Lrem - bat_profile->cell.Clow);

        V = (dL / grad_lower) + bat_profile->cell.Vlow;
    }
    else if (Lrem <= bat_profile->cell.Cmax)
    {
        float const dL = (Lrem - bat_profile->cell.Cmid);

        V = (dL / grad_upper) + bat_profile->cell.Vmid;
    }
    else
    {
        V = bat_profile->cell.Vmax;
    }

    return (V * (float)bat_profile->battery.series);
}
