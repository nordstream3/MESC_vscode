#ifndef BIT_OP_H
#define BIT_OP_H

#include <stdint.h>

#define BITS_PER_BYTE    UINT32_C(8)
#define BITS_PER_NYBBLE  UINT32_C(4)
#define NYBBLES_PER_BYTE (BITS_PER_BYTE / BITS_PER_NYBBLE)

#define BIT_MASK_32(bits)  (((UINT32_C(1) - ((bits) / 32)) << ((bits) & 31)) - UINT32_C(1))

#define BYTE_SHIFT(B) ((B) * BITS_PER_BYTE)

#endif
