/*
 * Instrument_cluster.c
 *
 *  Created on: 24-Jul-2025
 *      Author: 91626
 */
#include "sdk_project_config.h"
#include "gear_shifter.h"
#include "instrument_cluster.h"
#define DECAY_RATE 2
#define dx 0.2
extern rtc_timedate_t Current_Time;
uint8_t IC_cmd0[8],IC_cmd1[8];
extern volatile uint8_t drive_status,door_status,Swap_Flag,Tick_Flag,One_Sec_Done,Drive_Flag,Blinq_Swap;
extern uint8_t drive_mode;
uint8_t maintainence_status = 1,brake_warning_status = 1;
extern uint16_t speed_kmph,SOC_Battery,Capacity_Scaled;
uint8_t battery_soc =0x20,trip_distance =0x00,range =0x50,charge_hrs =0x03,\
	charge_mins=0x10,swap_mode_mins =0x5,swap_mode_secs =0x14;
uint16_t swap_seconds_left = SWAP_TOTAL_SECONDS,ic_update=0;
bool charge_mode = false,swap_mode = false;
extern uint8_t DL_cmd0[8], DL_cmd1[8],DL_cmd2[8],display_speed_can,brake_flag;
extern double odometer;
static uint8_t stable_drive_status;
extern volatile uint16_t vehicleSpeedKmph;
extern volatile uint8_t brake;
int brake_timer=0;
float displayed_speed=0;
extern volatile int16_t	DC_Current_Scaled;
void update_speed(void)
{
    // 1. Brake timer logic
    if (brake_flag)
    {
        brake_timer++;
    }
    else
    {
        brake_timer = 0;
    }
    // 2. Normal condition (no braking or short brake)
    if ( brake_timer<=500)
    {
     displayed_speed = vehicleSpeedKmph;
    }
    else
    {
        // 3. Apply gradual decay to zero
        if (displayed_speed > 0)
        {
        	float decay = displayed_speed * dx;

        	// Step 2: Subtract decay from current speed
        	displayed_speed = displayed_speed - decay;

            if (displayed_speed < 0)
                displayed_speed = 0;
        }
    }
}
void Swap_Countdown_Update(void)
{
    if (One_Sec_Done)
    {
    	One_Sec_Done = 0;
        if (swap_seconds_left > 0)
        {
            swap_seconds_left--;
    		Blinq_Swap=1;
        }
        else
        {
    		IC_cmd1[4] = 0x00;
        	Tick_Flag=0;
    		Blinq_Swap=0;
        }
        swap_mode_mins=swap_seconds_left/60;
        swap_mode_secs=swap_seconds_left%60;
    }
}
    void IC_update()
    {
    update_speed();
    stable_drive_status = drive_status;
	if(stable_drive_status == P)
	{
		IC_cmd0[0] = 0x00;
	}

	else if(stable_drive_status == D)
	{
		IC_cmd0[0] = 0x03;
	}

	else if(stable_drive_status == R)
	{
		IC_cmd0[0] = 0x02;
	}

	else if(stable_drive_status == N)
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
	if(DC_Current_Scaled>-10)
	{
		 displayed_speed=0;
	}
/*	if ((DC_Current_Scaled <= -1.0f) && (DC_Current_Scaled >= -10.0f))
	{
	    displayed_speed = 0;
	}
	if(DC_Current_Scaled)
	{
		  displayed_speed = 0;
	}*/
	IC_cmd0[1] =    displayed_speed;//vehicleSpeedKmph  ;  //display_speed_can  //displayed_speed
	trip_distance = (uint8_t) odometer;
	IC_cmd0[2] = 0x14;
	IC_cmd0[3] = 0x00;
	IC_cmd0[4] = 0x00;
	IC_cmd0[5] = SOC_Battery;
	IC_cmd0[6] = 0;
	IC_cmd0[7] =Capacity_Scaled*80/165; //range - trip_distance;
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
	 if(Blinq_Swap)
	{
		Tick_Flag=1;
		IC_cmd1[4] = 0x02;
		Swap_Countdown_Update();
		IC_cmd1[6] = swap_mode_mins;
		IC_cmd1[7] = swap_mode_secs;
	 }
	 else
	 {
		IC_cmd1[4] = 0x00;
    	swap_seconds_left = SWAP_TOTAL_SECONDS;
	 }
	 ic_update++;
}
void split_odometer()
{
	uint32_t odometer_final;
	odometer_final = (uint32_t)(odometer);
	IC_cmd0[2] = (odometer_final & 0xFF);
	IC_cmd0[3] = ((odometer_final >> 8) & 0xFF);
	IC_cmd0[4] = ((odometer_final >> 16) & 0xFF);
}
