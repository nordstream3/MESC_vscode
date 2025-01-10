#ifndef HASH_FNV_H
#define HASH_FNV_H

#include <stdint.h>

/*
REFERENCE

FNV-1A (Fowler/Noll/Vo) hash
http://www.isthe.com/chongo/tech/comp/fnv/index.html
Accessed 2021-04-17
*/

uint32_t fnv1a_data( void const * ptr, uint32_t const len );

uint32_t fnv1a_str( char const * ptr );

uint32_t fnv1a_init( void );

uint32_t fnv1a_process( uint32_t const fnv, uint8_t const byte );

uint32_t fnv1a_process_data( uint32_t const fnv, void const * ptr, uint32_t const len );

uint32_t fnv1a_process_zero( uint32_t const fnv, uint32_t const len );

#endif
