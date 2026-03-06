/*
 * Instrument_cluster.c
 *
 *  Created on: 24-Jul-2025
 *      Author: 91626
 */
#include "sdk_project_config.h"
#include "gear_shifter.h"
#include "instrument_cluster.h"

extern rtc_timedate_t Current_Time;
uint8_t IC_cmd0[8],IC_cmd1[8];
extern uint8_t drive_status,door_status;
extern uint8_t drive_mode;
uint8_t maintainence_status = 1,brake_warning_status = 1;
extern uint16_t speed_kmph;
uint8_t battery_soc =0x20,trip_distance =0x00,range =0x50,charge_hrs =0x03,\
	charge_mins=0x10,swap_mode_mins =0x5,swap_mode_secs =0x14;

bool charge_mode = false,swap_mode = false;
extern uint8_t DL_cmd0[8], DL_cmd1[8],DL_cmd2[8];
extern double odometer;

void IC_update()
{

	if(drive_status == P)
	{
		IC_cmd0[0] = 0x00;
	}

	else if(drive_status == D)
	{
		IC_cmd0[0] = 0x03;
	}

	else if(drive_status == R)
	{
		IC_cmd0[0] = 0x02;
	}

	else if(drive_status == N)
	{
		IC_cmd0[0] = 0x01;
	}

	if(drive_mode == ECO)
	{
		IC_cmd0[0]|=(0x02 << 3);

	}

	if(drive_mode == SPORTS)
	{
		IC_cmd0[0]|=(0x03 << 3);
	}

	IC_cmd0[0]|=(((brake_warning_status)) << 5);

	IC_cmd0[0]|= (maintainence_status << 6);

	IC_cmd0[0]|= (door_status << 7);

	IC_cmd0[1] = speed_kmph;

/**TODO:Split odometer based on calculated value upon dma callback*/
	//split_odometer();

/**Hardcode Odometer value*/
	trip_distance = (uint8_t) odometer;
	IC_cmd0[2] = 0x14;

	IC_cmd0[3] = 0x00;

	IC_cmd0[4] = 0x00;

	IC_cmd0[5] = battery_soc;

//	IC_cmd0[6] = trip_distance;
	IC_cmd0[6] = 0;

	IC_cmd0[7] = range - trip_distance;

	RTC_DRV_GetCurrentTimeDate(RTC_1, &Current_Time);

	IC_cmd1[0] = (uint8_t)(Current_Time.hour & 0xFF);

	IC_cmd1[1] = (uint8_t)(Current_Time.minutes & 0xFF);

	DL_cmd0[0] = (uint8_t)(Current_Time.hour & 0xFF);
	DL_cmd0[1] = (uint8_t)(Current_Time.minutes & 0xFF);
	DL_cmd0[2] = (uint8_t)(Current_Time.seconds & 0xFF);


	if(charge_mode)
	{
		IC_cmd1[4] = 0x01;

		IC_cmd1[6] = charge_hrs;

		IC_cmd1[7] = charge_mins;

	}
	else if(swap_mode)
	{
		IC_cmd1[4] = 0x02;

		IC_cmd1[6] = swap_mode_mins;

		IC_cmd1[7] = swap_mode_secs;
	}


}

void split_odometer()
{
	uint32_t odometer_final;

	odometer_final = (uint32_t)(odometer);
	IC_cmd0[2] = (odometer_final & 0xFF);

	IC_cmd0[3] = ((odometer_final >> 8) & 0xFF);

	IC_cmd0[4] = ((odometer_final >> 16) & 0xFF);

}
