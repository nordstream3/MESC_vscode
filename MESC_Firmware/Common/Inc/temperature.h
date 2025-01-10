#ifndef MESC_TEMP_H
#define MESC_TEMP_H

#include <stdbool.h>
#include <stdint.h>

enum TEMPMethod
{
    TEMP_METHOD_STEINHART_HART_BETA_R,
	TEMP_METHOD_KTY83_122_LINEAR,
	TEMP_METHOD_KTY84_130_LINEAR,
};

typedef enum TEMPMethod TEMPMethod;

enum TEMPSchema
{
    TEMP_SCHEMA_R_F_ON_R_T,
    TEMP_SCHEMA_R_T_ON_R_F
};

typedef enum TEMPSchema TEMPSchema;

struct TEMP
{
    float       V;
    float       R_F;

    uint32_t    adc_range;

    TEMPMethod  method;
    TEMPSchema  schema;

    union
    {
    struct
    {
    float       A;
    float       B;
    float       C;

    float       Beta;
    float       r;

    float       T0;
    float       R0;
    }           SH;
    }           parameters;

    struct
    {
    float       Tmin;
    float       Thot;
    float       Tmax;
    }           limit;
};

typedef struct TEMP TEMP;

float temp_read( TEMP const * const temp, uint32_t const adc_raw );

uint32_t temp_get_adc( TEMP const * const temp, float const T );

enum TEMPState
{
	TEMP_STATE_OK,
    TEMP_STATE_ROLLBACK,
	TEMP_STATE_OVERHEATED,
};

typedef enum TEMPState TEMPState;

TEMPState temp_check( TEMP const * const temp, float const T, float * const dT );

TEMPState temp_check_raw( TEMP const * const temp, uint32_t const adc_raw, float * const dT );

#endif
