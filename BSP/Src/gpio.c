/*
 * gpio.c
 *
 *  Created on: 17-Jul-2025
 *      Author: Rohit
 */
#include "sdk_project_config.h"
#include "gpio.h"
#include "gear_shifter.h"

void GPIO_Init()
{
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

	 /* Setup button pin */
//	PINS_DRV_SetPinsDirection(PTE, ~((1 << BTN1_PIN)|(1 << 16))); //configure PE15

	/** To configure PE15 as 5V input pin*/
	//PINS_DRV_SetPins(PTA, 1 << 14);

	//PINS_DRV_ClearPins(PTA, 1 << 11);

	/* Setup button pin */
//	PINS_DRV_SetPinsDirection(PTB, ~((1 << BTN2_PIN)|(1 << BTN3_PIN)|(1 << BTN4_PIN)|(1 << BTN5_PIN))); //configure PB8,9,10,11
	PINS_DRV_SetPinDirection(PTB, BTN2_PIN, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTB, BTN3_PIN, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTB, BTN4_PIN, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTB, BTN5_PIN, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTC,13, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTC,11, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTC,10, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTD,13, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTD,14, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTE, 15, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTE, 16, GPIO_INPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTE,9U, GPIO_OUTPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTD,15U, GPIO_OUTPUT_DIRECTION);
	PINS_DRV_SetPinDirection(PTD,0U, GPIO_OUTPUT_DIRECTION);


}
