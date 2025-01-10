#include "encoder.h"
#include "motorinstance.h"

/*#include "hw_setup.h"
#include "motor_control.h"
#include "sin_lut.h"
#include "motor.h"
#include "temperature.h"
#include "error.h"
#include "position.h"
#include "motorinstance.h"
#include "observers.h"
#include "input_vars.h"

#ifdef MESC_UART_USB
#include "usbd_cdc_if.h"
#endif

#include "conversions.h"

#include <math.h>
#include <stdlib.h>*/


uint8_t pkt_crc8(uint8_t crc /*CRC_SEED=0xFF*/, uint8_t *data, uint8_t length)
{
	int16_t i, bit;

	for (i = 0; i < length; i++)
	{
		crc ^= data[i];

		for (bit = 0; bit < 8; bit++)
		{
			if ((crc & 0x80) != 0)
			{
				crc <<= 1;
				crc ^= 0x1D; // CRC_POLYNOMIAL=0x1D;
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return crc;
}

struct __attribute__((__packed__)) SamplePacket
{
	struct
	{
		uint8_t crc;
		uint8_t STAT_RESP; // Should be 0xF_?
	} safetyword;
	uint16_t angle;
	int16_t speed;
	uint16_t revolutions;
};

typedef struct SamplePacket SamplePacket;
SamplePacket pkt;

void tle5012(MESC_motor_typedef *_motor)
{
#ifdef USE_SPI_ENCODER
	uint16_t const len = sizeof(pkt) / sizeof(uint16_t);
	uint16_t reg = (UINT16_C(1) << 15)	   /* RW=Read */
				   | (UINT16_C(0x0) << 11) /* Lock */
				   | (UINT16_C(0x0) << 10) /* UPD=Buffer */
				   | (UINT16_C(0x02) << 4) /* ADDR */
				   | (len - 1);			   /* ND */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)&reg, 1, 1000);
	HAL_SPI_Receive(&hspi3, (uint8_t *)&pkt, len, 1000);
	//      volatile uint8_t crc = 0;
	// #if 1
	//      reg ^= 0xFF00;
	//      crc = pkt_crc8( crc, &((uint8_t *)&reg)[1], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&reg)[0], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[1], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.angle)[0], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[1], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.speed)[0], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[1], 1 );
	//      crc = pkt_crc8( crc, &((uint8_t *)&pkt.revolutions)[0], 1 );
	// #else
	//      crc = pkt_crc8( crc, &reg, 2 );
	//      crc = pkt_crc8( crc, &pkt.angle, 6 );
	// #endif
	//      crc = pkt_crc8( crc, &pkt.safetyword.STAT_RESP, 1 );
	//      crc = ~crc;
	//      if (crc != pkt.safetyword.crc)
	//      {
	//    	  __NOP();
	//    	  __NOP();
	//    	  __NOP();
	//      }
	//      else
	//      {
	//    	  __NOP();
	//      }

	pkt.angle = pkt.angle & 0x7fff;
#ifdef ENCODER_DIR_REVERSED
	_motor->FOC.enc_angle = -POLE_PAIRS * ((pkt.angle * 2) % POLE_ANGLE) - _motor->FOC.enc_offset;
#else
	_motor->FOC.enc_angle = POLE_PAIRS * ((pkt.angle * 2) % POLE_ANGLE) - _motor->FOC.enc_offset;
#endif
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	pkt.revolutions = pkt.revolutions & 0b0000000111111111;
#endif
}

void getIncEncAngle(MESC_motor_typedef *_motor)
{
	if (_motor->FOC.encoder_polarity_invert)
	{
		_motor->FOC.enc_angle = _motor->m.pole_pairs * (65536 - (_motor->FOC.enc_ratio * (uint16_t)_motor->enctimer->Instance->CNT - _motor->FOC.enc_ratio * (uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;
	}
	else
	{
		_motor->FOC.enc_angle = _motor->m.pole_pairs * ((_motor->FOC.enc_ratio * (uint16_t)_motor->enctimer->Instance->CNT - _motor->FOC.enc_ratio * (uint16_t)_motor->enctimer->Instance->CCR3)) + _motor->FOC.enc_offset;
	}
}
