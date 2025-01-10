#ifndef FINGERPRINT_H
#define FINGERPRINT_H

#include "string_op.h"

#define MESC_TIMESTAMP_YEAR   MAKE_UINT32_STRING('2','0','2','2')
#define MESC_TIMESTAMP_MONTH  MAKE_UINT16_STRING('0','2')
#define MESC_TIMESTAMP_DAY    MAKE_UINT16_STRING('0','6')
#define MESC_TIMESTAMP_HOUR   MAKE_UINT16_STRING('1','4')
#define MESC_TIMESTAMP_MINUTE MAKE_UINT16_STRING('4','6')
#define MESC_GITHASH_WORDS (160 / 32)
#define MESC_GITHASH {UINT32_C(0xc5f8904f),UINT32_C(0x0593d19b),UINT32_C(0x67d05c7b),UINT32_C(0xdd130ad3),UINT32_C(0x837addf0)}

struct MESCFingerprint
{
    uint32_t    year;              // Timestamp (MESC_TIMESTAMP_YEAR)
    uint16_t    month;             // Timestamp (MESC_TIMESTAMP_MONTH)
    uint16_t    day;               // Timestamp (MESC_TIMESTAMP_DAY)

    uint16_t    hour;              // Timestamp (MESC_TIMESTAMP_HOUR)
    uint16_t    minute;            // Timestamp (MESC_TIMESTAMP_MINUTE)

    uint8_t     _zero;             // Must be zero
    uint8_t     reserved[3];

    uint32_t    githash[MESC_GITHASH_WORDS]; // Git hash of firmware (MESC_GITHASH)
};

typedef struct MESCFingerprint MESCFingerprint;

#define MESC_FINGERPRINT {MESC_TIMESTAMP_YEAR,MESC_TIMESTAMP_MONTH,MESC_TIMESTAMP_DAY,MESC_TIMESTAMP_HOUR,MESC_TIMESTAMP_MINUTE,0,{0,0,0},MESC_GITHASH}

#endif
