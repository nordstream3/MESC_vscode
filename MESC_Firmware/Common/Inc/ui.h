#ifndef UI_H
#define UI_H

#include <stdint.h>

#define UI_PROFILE_SIGNATURE MAKE_UINT32_STRING('M','U','P','E')

enum UIProfileType
{
    // Inputs
    UI_PROFILE_THROTTLE,
    UI_PROFILE_BRAKE,
    UI_PROFILE_BUTTON,

    // Outputs
    UI_PROFILE_INDICATOR,
    UI_PROFILE_SCREEN,
};

typedef enum UIProfileType UIProfileType;

enum UIResponse
{
    UI_RESPONSE_LINEAR,
    UI_RESPONSE_LOG,
};

typedef enum UIResponse UIResponse;

enum UIActivation
{
    UI_ACTIVATION_LEVEL,
    UI_ACTIVATION_EDGE,
};

typedef enum UIActivation UIActivation;

struct UIProfile
{
    UIProfileType   type;

    union
    {
    // Inputs
    struct
    {
    uint32_t        adc_min;
    uint32_t        adc_max;
    UIResponse      response;
    uint32_t        adc_trig;
    float           Imax;
    uint32_t        rcpwm_t_min;
    uint32_t        rcpwm_t_max;
    }               throttle;

    struct
    {
    uint32_t        adc_min;
    uint32_t        adc_max;
    UIResponse      response;
    uint32_t        adc_trig;
    float           Imax;
    uint32_t        _[2];
    }               brake;

    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        identifier;
    uint32_t        _[4];
    }               button;

    // Outputs
    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        identifier;
    UIActivation    activation;
    uint32_t        _[3];
    }               indicator;

    struct
    {
    uint32_t        interface;
    uint32_t        address;
    uint32_t        width;
    uint32_t        height;
    uint32_t        _[3];
    }               screen;

    }               desc;
};

typedef struct UIProfile UIProfile;

void ui_init( UIProfile const * const profile );

#endif
