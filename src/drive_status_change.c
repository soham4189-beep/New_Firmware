/*
 * drive_state_change.c
 *
 *  Created on: 02-Jul-2025
 *      Author: Rohit
 */
#include"stdio.h"
#include "gear_shifter.h"
#include "sdk_project_config.h"
uint8_t drive_status = P; /** Denotes current state of car.It is initialised in parking mode*/
uint8_t gear_state = PARKING,brake = OFF,error;
uint16_t speed_rpm_motor,wheel_rpm,speed_kmph=100;
uint8_t speed =0;
extern int throttle_data, neutral_count;
extern uint8_t mc_cmd[8];
uint8_t drive_mode = ECO;
uint8_t door_status = 1;
extern volatile uint8_t Drive_Enable,Drive_Flag,drive_state_change_Flag;
void drive_state_change()
{
if(drive_state_change_Flag)
{
if(Drive_Flag)
{
	if(gear_state == DRIVE && speed < 1.3 &&\
			throttle_data == 0 )
	{
		PINS_DRV_ClearPins(PTB,1<< 4);
	   	delay(0x83D600);
		PINS_DRV_SetPins(PTD,1<<1);
		drive_status = D;
		PINS_DRV_ClearPins(PTE, 1 << 9);
        mc_cmd[0] =MC_FORWARD_STATE;
		mc_cmd[1] = 0x00;//Reserved
		mc_cmd[2] = 0x00;//Reserved
		mc_cmd[3] = TORQUE_MODE;
	}
}
if(Drive_Flag)
{
	 if(gear_state == REVERSE && speed < 1.3 && throttle_data == 0 )
	{
		PINS_DRV_ClearPins(PTD,1 <<1);
		delay(0x83D600);
		PINS_DRV_SetPins(PTB,1 << 4);
		drive_status = R;
		PINS_DRV_SetPins(PTE, 1 << 9);
        mc_cmd[0] =MC_BACKWARD_STATE;
		mc_cmd[1] = 0x00;//Reserved
		mc_cmd[2] = 0x40;//Reserved
		mc_cmd[3] = TORQUE_MODE;

	}
}
if(	Drive_Flag)
{
	 if(gear_state == NEUTRAL && gear_state != REVERSE
			&& gear_state != DRIVE && gear_state != PARKING)
	{
		PINS_DRV_ClearPins(PTB,1 << 4);
		PINS_DRV_ClearPins(PTD,1 << 1);
		drive_status = N;
		PINS_DRV_ClearPins(PTE, 1 << 9);
		mc_cmd[0] = MC_FORWARD_STATE;//MC_PARKING_STATE;
		mc_cmd[1] = 0x00;//Reserved
		mc_cmd[2] = 0x00;//Reserved
		mc_cmd[3] = TORQUE_MODE;
		mc_cmd[4] = 0x00;
		mc_cmd[5] = 0x00;
		mc_cmd[6] = 0x00;
		mc_cmd[7] = 0x00;
	}
}

if(gear_state == PARKING )
{

		PINS_DRV_ClearPins(PTB,1 << 4);
		PINS_DRV_ClearPins(PTD,1 << 1);
		drive_status = P;
		PINS_DRV_ClearPins(PTE, 1 << 9);
        mc_cmd[0] = MC_PARKING_STATE;
		mc_cmd[1] = 0x00;//Reserved
		mc_cmd[2] = 0x00;//Reserved
		mc_cmd[3] = TORQUE_MODE;
		mc_cmd[4] = 0x00;
		mc_cmd[5] = 0x00;
		mc_cmd[6] = 0x00;
		mc_cmd[7] = 0x00;
}
}
drive_state_change_Flag=0;
}
