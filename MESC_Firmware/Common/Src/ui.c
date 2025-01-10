#include "ui.h"
#include "command_line_interface.h"
#include "profile.h"

#define UI_PROFILE_MAX_ENTRIES UINT32_C(8)

static UIProfile    ui_profile[UI_PROFILE_MAX_ENTRIES];
static char const * ui_profile_name[UI_PROFILE_MAX_ENTRIES];

void ui_init( UIProfile const * const profile )
{
    if (profile == PROFILE_DEFAULT)
    {
        uint32_t i = 0;
        uint32_t d = 0;
        uint32_t ui_length = sizeof(UIProfile);

        do
        {
            ProfileStatus const ret = profile_scan_entry(
                &i, UI_PROFILE_SIGNATURE,
                &ui_profile[d], &ui_length,
                &ui_profile_name[d] );

            if (ret != PROFILE_STATUS_SUCCESS)
            {
                return;
            }

            cli_reply( "UI ADD %d" "\r" "\n", ui_profile[d].type );

            ui_init( &ui_profile[d] );
            d++;
        }
        while (d < UI_PROFILE_MAX_ENTRIES);
    }

    switch (profile->type)
    {
    // Inputs
        case UI_PROFILE_THROTTLE:
            //profile->desc.throttle;
            break;
        case UI_PROFILE_BRAKE:
            //profile->desc.brake;
            break;
        case UI_PROFILE_BUTTON:
            //profile->desc.button;
            break;
    // Outputs
        case UI_PROFILE_INDICATOR:
            //profile->desc.indicator;
            break;
        case UI_PROFILE_SCREEN:
            //profile->desc.screen;
            break;
        default:
            break;
    }
}
