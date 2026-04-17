/*
 * gear_shifter.h
 *
 *  Created on: 05-Jul-2025
 *      Author: Rohit
 */

#ifndef INC_GEAR_SHIFTER_H_
#define INC_GEAR_SHIFTER_H_

#include"stdio.h"
#define SPEED_MODE 					0xC0
#define THROTTLE_MODE 				0x50
#define TORQUE_MODE 				0xA0
#define THROTTLE_CONTROL 			0x00
#define MC_BACKWARD_STATE			0x06
#define MC_FORWARD_STATE  			0x05
#define MC_PARKING_STATE     		0x04


#endif
typedef enum{
	DRIVE = 0,//0
	NEUTRAL,//1
	PARKING,//2
	REVERSE//3
}state;

typedef enum{
	D = 0,//0
	N,//1
	P,//2
	R//3
}drive_state;

typedef enum{
	ON =0,
	OFF//brake is active low
}brake_state;
typedef enum{
	ECO =0,
	SPORTS//brake is active low
}mode_drive;
typedef enum
{
SYS_IDLE = 0,
SYS_DRIVE_ACTIVE,
SYS_SWAP_ACTIVE
} system_state_t;
extern volatile system_state_t system_state;
void drive_state_change(void);
void Gear_Interrupt_init(void);
void PortE_ISR(void);
void PortB_ISR(void);
void PortC_ISR(void);
