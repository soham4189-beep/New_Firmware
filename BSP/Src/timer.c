/*
 * timer.c
 *
 *  Created on: 17-Jul-2025
 *      Author: Rohit
 */
#include "sdk_project_config.h"
#include "timer.h"
#include "interrupt_manager.h"
#include "instrument_cluster.h"
#include "CAN.h"
#include "gear_shifter.h"
#include "string.h"
#define Current_Active_Threshold 5
//Model
#define BRAKE_ON        1
#define ALPHA_BRAKE     0.8f
#define ALPHA_NORMAL    0.3f
#define BETA_CORRECTION 0.2f   // Drift correction gain
extern volatile uint8_t neutral_flag,Debounce_Check_Flag,Drive_Pin_Status,Swap_Pin_Status,Swap_Flag,Drive_Flag,g_canTx2Done,Gear_Shifter_DebounceCheck;
extern volatile  uint8_t  IC_cmd0[8],IC_cmd1[8],g_canTx1Done,drive_state_change_Flag, gear_state,Drive_Flag,Mode_Change_Flag,Indicator_On,bmsRequestIndex;
extern volatile bool waitingResponse,g_canRx0Done,g_canRx1Done,g_canRx2Done,g_canRx3Done,g_canRx4Done,g_canRx5Done,g_canRx6Done,g_canRx7Done,g_canRx8Done,g_canRx9Done;
extern int  neutral_count;
#define LPIT_CHANNEL 0U
#define LPIT_CHANNEL_1 1U
int Tick=0,Chicago=0,Tick1=0,variable=0,previous_value=0,decimal_part=0;
extern volatile int16_t DC_Current_Scaled;
float timer_in_sec=0 ,timer_counter=0;
uint8_t Tick_Flag=0,One_Sec_Done=0,Gear_State_Change=0,Debounce_Tick,display_speed_can;
char tx_buffer[256];
extern volatile uint16_t vehicleSpeedKmph;
float K = 0.03;   // tune this
extern volatile uint8_t log_data,brake;
extern volatile float Cell_Voltage_1_Scaled,Cell_Voltage_2_Scaled,Cell_Voltage_3_Scaled,Cell_Voltage_4_Scaled,Cell_Voltage_5_Scaled,Cell_Voltage_6_Scaled,Cell_Voltage_7_Scaled,Cell_Voltage_8_Scaled,Cell_Voltage_9_Scaled,Cell_Voltage_10_Scaled,Cell_Voltage_11_Scaled,Cell_Voltage_12_Scaled,Cell_Voltage_13_Scaled,Cell_Voltage_14_Scaled,Cell_Voltage_15_Scaled,Cell_Voltage_16_Scaled;
extern volatile uint16_t raw_voltage,Capacity,Raw_Voltage_Scaled,Capacity_Scaled,Cell_Voltage_1,Cell_Voltage_2,Cell_Voltage_3,Cell_Voltage_4,Cell_Voltage_5,Cell_Voltage_6,Cell_Voltage_7,Cell_Voltage_8,Cell_Voltage_9,Cell_Voltage_10,Cell_Voltage_11,Cell_Voltage_12,Cell_Voltage_13,Cell_Voltage_14,Cell_Voltage_15,Cell_Voltage_16,SOC_Battery,DC_Current,charge_mos,ntc_1,ntc_2,ntc_1_scaled,ntc_2_scaled,vehicleSpeedKmph;
const uint16_t bmsReqList[] =
{
	0x100,
    0x107,
    0x108,
    0x109,
    0x10A,
    0x10B,
    0x102,
    0x10D
};
#define TOTAL_BMS_REQ  8
void UpdateSpeedEstimate(float DC_Current_Scaled,uint16_t vehicleSpeedKmph,float dt,int brake,float Current_Threshold)
{
    static float v_est = 0.0f;
    static float v_prev = 0.0f;
    float gps_speed=(float)vehicleSpeedKmph;
    float a_est, alpha, error;
    float display_speed_f;

    int braking_detected;
    // --------- 1. Braking Detection ---------
    braking_detected = (DC_Current_Scaled > Current_Threshold || brake == BRAKE_ON);
    // --------- 2. Acceleration Estimation ---------
    a_est = K * DC_Current_Scaled;
    // --------- 3. Prediction ---------
    v_est = v_prev + (a_est * dt);
    // --------- 4. Drift Correction ---------
    error = gps_speed - v_est;
    v_est += BETA_CORRECTION * error;
    // --------- 5. Alpha Selection ---------
    if (braking_detected)
    {
        alpha = ALPHA_BRAKE;
    }
    else
    {
        alpha = ALPHA_NORMAL;
    }
    // --------- 6. Fusion ---------
    display_speed_f = (alpha * v_est) + ((1.0f - alpha) * gps_speed);
    // --------- 7. Clamp ---------
    if (v_est < 0.0f)
        v_est = 0.0f;
    // --------- 8. Update State ---------
    v_prev = v_est;
    // --------- 9. Convert to uint8_t (WHOLE NUMBER km/h) ---------
    int temp_speed = (int)(display_speed_f + 0.5f);  // rounding
    if (temp_speed < 0)
        temp_speed = 0;
    else if (temp_speed > 255)
        temp_speed = 255;
    display_speed_can = (uint8_t)temp_speed;
}
void BMS_Request_Manager(void)
{
	if(g_canRx0Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
	    	raw_voltage=(((uint16_t)recvBuff306.data[0] << 8)|(uint16_t)recvBuff306.data[1]);
	    	DC_Current=(((uint16_t)recvBuff306.data[2] << 8)|(uint16_t)recvBuff306.data[3]);
	    	Capacity  =(((uint16_t)recvBuff306.data[4] << 8) |(uint16_t)recvBuff306.data[5]);
	    	Raw_Voltage_Scaled=raw_voltage*0.01;
	    	Capacity_Scaled=Capacity*0.01;
	    	DC_Current_Scaled=(int16_t) DC_Current*0.01;
            BMS_Req(TX_MAILBOX2,0x107,NULL,0U);
	}
	if(g_canRx1Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
	    	Cell_Voltage_1 =(((uint16_t)recvBuff307.data[0]<<8)|(uint16_t)recvBuff307.data[1]);
	    	Cell_Voltage_1_Scaled=(float)Cell_Voltage_1*0.001;
	    	Cell_Voltage_2 =(((uint16_t)recvBuff307.data[2]<<8)|(uint16_t)recvBuff307.data[3]);
	    	Cell_Voltage_2_Scaled=(float)Cell_Voltage_2*0.001;
	    	Cell_Voltage_3 =(((uint16_t)recvBuff307.data[4]<<8)|(uint16_t)recvBuff307.data[5]);
	        Cell_Voltage_3_Scaled=(float)Cell_Voltage_3*0.001;
            BMS_Req(TX_MAILBOX2,0x108,NULL,0U);
	}
	if(g_canRx2Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
	    	Cell_Voltage_4 =(((uint16_t)recvBuff308.data[0]<<8)|(uint16_t)recvBuff308.data[1]);
	    	Cell_Voltage_4_Scaled= (float)Cell_Voltage_4*0.001;
	    	Cell_Voltage_5 =(((uint16_t)recvBuff308.data[2]<<8)|(uint16_t)recvBuff308.data[3]);
	    	Cell_Voltage_5_Scaled=(float)Cell_Voltage_5*0.001;
	    	Cell_Voltage_6 =(((uint16_t)recvBuff308.data[4]<<8)|(uint16_t)recvBuff308.data[5]);
	    	Cell_Voltage_6_Scaled=(float)Cell_Voltage_6*0.001;
            BMS_Req(TX_MAILBOX2,0x109,NULL,0U);
	}
	if(g_canRx3Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x10A,NULL,0U);
           	Cell_Voltage_7 =(((uint16_t)recvBuff309.data[0]<<8)|(uint16_t)recvBuff309.data[1]);
            Cell_Voltage_7_Scaled = (float)Cell_Voltage_7*0.001;
            Cell_Voltage_8 =(((uint16_t)recvBuff309.data[2]<<8)|(uint16_t)recvBuff309.data[3]);
            Cell_Voltage_8_Scaled=(float)Cell_Voltage_8*0.001;
            Cell_Voltage_9 =(((uint16_t)recvBuff309.data[4]<<8)|(uint16_t)recvBuff309.data[5]);
            Cell_Voltage_9_Scaled=(float)Cell_Voltage_9*0.001;
	}
	if(g_canRx4Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x10B,NULL,0U);
        	Cell_Voltage_10 =(((uint16_t)recvBuff310.data[0]<<8)|(uint16_t)recvBuff310.data[1]);
        	Cell_Voltage_10_Scaled=(float) Cell_Voltage_10*0.001;
        	Cell_Voltage_11 =(((uint16_t)recvBuff310.data[2]<<8)|(uint16_t)recvBuff310.data[3]);
        	Cell_Voltage_11_Scaled=(float)Cell_Voltage_11*0.001;
        	Cell_Voltage_12 =(((uint16_t)recvBuff310.data[4]<<8)|(uint16_t)recvBuff310.data[5]);
        	Cell_Voltage_12_Scaled=(float)Cell_Voltage_12*0.001;
	}
	if(g_canRx5Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x10C,NULL,0U);
        	Cell_Voltage_13 = (((uint16_t)recvBuff311.data[0]<<8)|(uint16_t)recvBuff311.data[1]);
        	Cell_Voltage_13_Scaled=(float)Cell_Voltage_13*0.001;
        	Cell_Voltage_14= (((uint16_t)recvBuff311.data[2]<<8)|(uint16_t)recvBuff311.data[3]);
        	Cell_Voltage_14_Scaled=(float)Cell_Voltage_14*0.001;
        	Cell_Voltage_15= (((uint16_t)recvBuff311.data[4]<<8)|(uint16_t)recvBuff311.data[5]);
        	Cell_Voltage_15_Scaled=(float)Cell_Voltage_15*0.001;
	}
	if(g_canRx6Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x102,NULL,0U);
	    	Cell_Voltage_16= (((uint16_t)recvBuff312.data[0]<<8)|(uint16_t)recvBuff312.data[1]);
	    	Cell_Voltage_16_Scaled=(float)Cell_Voltage_16*0.001;
	}
	if(g_canRx7Done)
	{
		    //This ID is for BMS Errors we can log any BMS errors
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x103,NULL,0U);

	}
	if(g_canRx8Done)
	{
            g_canRx0Done=false;
			g_canRx1Done=false;
			g_canRx2Done=false;
			g_canRx3Done=false;
			g_canRx4Done=false;
			g_canRx5Done=false;
			g_canRx6Done=false;
			g_canRx7Done=false;
			g_canRx8Done=false;
			g_canRx9Done=false;
            BMS_Req(TX_MAILBOX2,0x105,NULL,0U);
	    	charge_mos=(((uint16_t)recvBuff314.data[0]<<8)|(uint16_t)recvBuff314.data[1]);
	}
	if(g_canRx9Done)
	{
		        g_canRx0Done=false;
				g_canRx1Done=false;
				g_canRx2Done=false;
				g_canRx3Done=false;
				g_canRx4Done=false;
				g_canRx5Done=false;
				g_canRx6Done=false;
				g_canRx7Done=false;
				g_canRx8Done=false;
				g_canRx9Done=false;
		        BMS_Req(TX_MAILBOX2,0x100,NULL,0U);
		    	ntc_1=(((uint16_t)recvBuff315.data[0]<<8)|(uint16_t)recvBuff315.data[1]);
		    	ntc_1_scaled=ntc_1*0.01;
				ntc_2=(((uint16_t)recvBuff315.data[4]<<8)|(uint16_t)recvBuff315.data[5]);
				ntc_2_scaled=ntc_2*0.01;
	}

/*    if(waitingResponse == false)
    {
          BMS_Req(TX_MAILBOX2,0x100,NULL,0U);
   //     Response_Received=false;
   //     bmsTimeoutCounter = 0;
    }
    else
    {
    	bmsRequestIndex++;
    	if(bmsRequestIndex >= TOTAL_BMS_REQ)  bmsRequestIndex=0;
        waitingResponse = false;
    }*/
}
void UART_Send(char *data)
{
LPUART_DRV_SendDataBlocking(1,(uint8_t *)data,strlen(data),100);
}
void Log_BMS_Data(void)
{
    snprintf(tx_buffer, sizeof(tx_buffer),"%d,%d,%d,\r\n",
    (int)vehicleSpeedKmph,
    (int)(DC_Current_Scaled),
	timer_in_sec
    );
    /*    "%d,%d,%d,%d,%d,%d,%d,%d,"
    "%d,%d,%d,%d,%d,%d,%d,%d,"
    "%d,%d,%d*/
  /*  (int)(Cell_Voltage_1_Scaled * 1000),
    (int)(Cell_Voltage_2_Scaled * 1000),
    (int)(Cell_Voltage_3_Scaled * 1000),
    (int)(Cell_Voltage_4_Scaled * 1000),
    (int)(Cell_Voltage_5_Scaled * 1000),
    (int)(Cell_Voltage_6_Scaled * 1000),
    (int)(Cell_Voltage_7_Scaled * 1000),
    (int)(Cell_Voltage_8_Scaled * 1000),
    (int)(Cell_Voltage_9_Scaled * 1000),
    (int)(Cell_Voltage_10_Scaled * 1000),
    (int)(Cell_Voltage_11_Scaled * 1000),
    (int)(Cell_Voltage_12_Scaled * 1000),
    (int)(Cell_Voltage_13_Scaled * 1000),
    (int)(Cell_Voltage_14_Scaled * 1000),
    (int)(Cell_Voltage_15_Scaled * 1000),
    (int)(Cell_Voltage_16_Scaled * 1000),
    (int)(ntc_1_scaled * 100),
    (int)(ntc_2_scaled * 100),
	charge_mos*/

    UART_Send(tx_buffer);
}

void PDB1_Trigger_Timer_Init()
{
	LPTMR_DRV_Init(INST_LPTMR_1, &lptmr_1_config0, false);
	/* Install IRQ handler for LPTMR interrupt */
	INT_SYS_InstallHandler(LPTMR0_IRQn, &lptmrISR, ((void *)0));
	/* Enable IRQ for LPTMR */
	INT_SYS_EnableIRQ(LPTMR0_IRQn);
	/* Start LPTMR counter */
	LPTMR_DRV_StartCounter(INST_LPTMR_1);
}

void lptmrISR(void)
{
    LPTMR_DRV_ClearCompareFlag(INST_LPTMR_1);
    Chicago++;
    BMS_Request_Manager();
 //   timer_counter++;
//    timer_in_sec = (float) timer_counter*0.560;

//    IC_CANTx();
}
void LPIT_ISR(void)
{
 //   Log_BMS_Data();
LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT_CONFIG_1, (1 << LPIT_CHANNEL));
if(Tick_Flag)
{
Tick++;
}
if(Debounce_Check_Flag)
{
Debounce_Tick++;
}
if(Debounce_Tick>=25)
{
Debounce_Tick=0;
Debounce_Check_Flag=0;
Mode_Change_Flag=1;
}
if(Tick>=120)
{
One_Sec_Done=1;
Tick=0;
}
}
void LPIT_ISR_1 (void)
{
	variable++;
//	UpdateSpeedEstimate(DC_Current_Scaled,vehicleSpeedKmph,0.1, brake,Current_Active_Threshold);
	//timer_counter=(float)variable*0.1;
//	decimal_part=timer_counter%100;
   // timer_in_sec=timer_counter+previous_value;
   // previous_value=	variable;
	log_data=1;
	LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT_CONFIG_1, (1 << LPIT_CHANNEL_1));

}
void LPIT_Init(void)
{
	status_t status;
	LPIT_DRV_Init(INST_LPIT_CONFIG_1, &lpit1_InitConfig);
    /* Initialize LPIT channel 0 and configure it as a periodic counter
    * which is used to generate an interrupt every second.*/
	status = LPIT_DRV_InitChannel(INST_LPIT_CONFIG_1, LPIT_CHANNEL, &lpit1_ChnConfig0);
	DEV_ASSERT(status == STATUS_SUCCESS);
	status = LPIT_DRV_InitChannel(INST_LPIT_CONFIG_1,  LPIT_CHANNEL_1, &lpit1_ChnConfig1);
	DEV_ASSERT(status == STATUS_SUCCESS);

	/* Install LPIT_ISR as LPIT interrupt handler */
	INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, &LPIT_ISR, NULL);
	INT_SYS_InstallHandler(LPIT0_Ch1_IRQn, &LPIT_ISR_1, NULL);

	/* Start LPIT0 channel 0 counter */
	LPIT_DRV_StartTimerChannels(INST_LPIT_CONFIG_1,
	                           (1 << LPIT_CHANNEL) | (1 << LPIT_CHANNEL_1));
}

