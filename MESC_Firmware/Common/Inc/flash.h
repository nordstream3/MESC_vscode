#ifndef FLASH_H_
#define FLASH_H_

#include "profile.h"

/*
TODO: Define in hw_setup.c
BEGIN
*/
uint32_t getFlashBaseAddress( void );
uint32_t getFlashBaseSize( void );
ProfileStatus eraseFlash( uint32_t const address, uint32_t const length );
/*
END
*/

void flash_register_profile_io( void );

#endif /* INC_MESCFLASH_H_ */
