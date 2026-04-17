/*!
** Copyright 2020 NXP
** @file main.c
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */
/* Including necessary configuration files.*/
#include "sdk_project_config.h"
volatile int exit_code = 0;
#include "stdio.h"
#include "gear_shifter.h"
#include "Throttle_read.h"
#include "interrupt_manager.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "dma.h"
#include "CAN.h"
#include "rtc.h"
#include "instrument_cluster.h"
#include "pins_driver.h"
#include "string.h"
#include <stdlib.h>
#define PTE16_MASK (1UL << 16)
#define ON  1
#define OFF 0
//#define Current_Threshold 5
#define RX_BUFFER_SIZE 512
#define DECAY_RATE 2.0    // kmph per second (tune this)
int a=0,transmit_count=0,loop_count=0;
extern float timer_in_sec,timer_counter;
float Cell_Voltage_1_Scaled=0,Cell_Voltage_2_Scaled=0,Cell_Voltage_3_Scaled=0,Cell_Voltage_4_Scaled=0,Cell_Voltage_5_Scaled=0,Cell_Voltage_6_Scaled=0,Cell_Voltage_7_Scaled=0,Cell_Voltage_8_Scaled=0,Cell_Voltage_9_Scaled=0,Cell_Voltage_10_Scaled=0,Cell_Voltage_11_Scaled=0,Cell_Voltage_12_Scaled=0,Cell_Voltage_13_Scaled=0,Cell_Voltage_14_Scaled=0,Cell_Voltage_15_Scaled=0,Cell_Voltage_16_Scaled=0;
uint16_t delayValue= 0,raw_voltage=0,Capacity=0,Raw_Voltage_Scaled=0,Capacity_Scaled=0,Cell_Voltage_1=0,Cell_Voltage_2=0,Cell_Voltage_3=0,Cell_Voltage_4=0,Cell_Voltage_5=0,Cell_Voltage_6=0,Cell_Voltage_7=0,Cell_Voltage_8=0,Cell_Voltage_9=0,Cell_Voltage_10=0,Cell_Voltage_11=0,Cell_Voltage_12=0,Cell_Voltage_13=0,Cell_Voltage_14=0,Cell_Voltage_15=0,Cell_Voltage_16=0,SOC_Battery=0,DC_Current=0;
uint8_t mc_cmd[8],transmit_flag=0,HV_Interlock=0,Swap_Done=0,HV_Interlock_2=0;
uint8_t bms_cmd[8],	neutral_flag=0,Drive_Pin_Status=0,Swap_Pin_Status=0,log_data=0;
uint8_t DL_cmd0[8],DL_cmd1[8],DL_cmd2[8],counter_flag=0,BMS_Command_Off_Flag=1,BMS_Command_On_Flag=0,Mode_Change_Flag=0, Digital_Read_1=0,Digital_Read_3=0;
volatile bool g_canTx1Done=false,g_canTx2Done=true,g_canTx3Done=false,g_canTx4Done=false,g_canTx5Done=false,g_canRx0Done=false,g_canRx1Done=false,g_canRx2Done=false,g_canRx3Done=false,g_canRx4Done=false,g_canRx5Done=false,g_canRx6Done=false,g_canRx7Done=false,g_canRx8Done=false,g_canRx9Done=false;
int neutral_count=0,count=0,uart=0;
extern volatile uint8_t Drive_Flag,Swap_Flag,gear_state,drive_state_change_Flag,swap_seconds_left,Gear_State_Change;
extern  uint8_t IC_cmd0[8],IC_cmd1[8],drive_status;
//flexcan_msgbuff_t recvBuff301,recvBuff302,recvBuff303,recvBuff304,recvBuff305,recvBuff306,recvBuff307,recvBuff308,recvBuff309,recvBuff310,recvBuff311,recvBuff312,recvBuff313,recvBuff314,recvBuff315,recvBuff316;
MYADCDMA0 ADC1_Sample;
volatile system_state_t system_state = SYS_IDLE;
uint8_t buffer[255];
uint8_t rxByte;
int constant=0,b=0,c=0,d=0,e=0,top=0,f=0,g=0,h=0;
#define NMEA_MAX_LEN 128
static char nmeaSentence[NMEA_MAX_LEN];
static uint16_t nmeaIndex = 0;
volatile bool g_bmsTxFree = true;
volatile bool nmeaReady = false;
volatile bool dataReady = false;
uint32_t bmsTimeoutCounter = 0,timeoutCounter=0,comeback=0;
float speed_knots = 0.0f;
float speed_kmph1  = 0.0f;
volatile char nmeaLine[NMEA_MAX_LEN];
uint16_t nmeaIdx = 0,uart_frequency=0,vehicleSpeedKmph=1,Full_Capacity=0,RSOC=0,Discharge_Cycle=0,charge_mos=0,ntc_1=0,ntc_2=0,ntc_1_scaled=0,ntc_2_scaled=0;
volatile char nmeaBuf[NMEA_MAX_LEN];
volatile bool sentenceStarted = false;
uint8_t ignition,check,command_off=1,command_on=1,Blinq_Swap=0,vehicle_restart=0,braking_detected=0,Key_Switch_Counter_Flag=0,latch=1,latch_2=1;
volatile uint8_t acc=0;
float speedKnots = 0.0f,alpha=0;
float a_est = 0;
#define BMS_IDLE  0
#define BMS_CMD_ON   1
#define BMS_CMD_OFF  2
#define BMS_TIMEOUT_LIMIT 1000U
volatile uint8_t bms_cmd_request = BMS_CMD_OFF;
volatile bool bmsSendActive = false;
static uint8_t last_cmd_sent = BMS_IDLE;
volatile uint8_t bmsRequestIndex = 0;
volatile bool waitingResponse = false,Response_Received=false;
int t=0,Key_Switch_Counter=0;
int16_t DC_Current_Scaled=1;
extern volatile int decimal_part,variable;
char logBuffer[200];
typedef enum
{
ACC,
IGNITION
}CarStatus_t;
CarStatus_t car_status;
typedef enum
{
VTG_IDLE,
VTG_V_FOUND,
VTG_T_FOUND
}vtg_state_t;
volatile vtg_state_t vtgState = VTG_IDLE;
volatile bool vtgDetected = false;
void Clock_init(void)
{
/* Write your local variable definition here */
status_t status;
/* Initialize clock module */
status = CLOCK_DRV_Init(&clockMan1_InitConfig0);
DEV_ASSERT(status == STATUS_SUCCESS);
(void) status;
}

void BMS_Command_Off(void)
{
	bms_cmd[0] = 1;
	bms_cmd[1] = 254;
	bms_cmd[2] = 1;
	bms_cmd[3] = 254;
	SendCANData_BMS(TX_MAILBOX,0x12D,bms_cmd,8UL);

}
void BMS_Command_On(void)
{
    bms_cmd[0] = 0;
    bms_cmd[1] = 255;
    bms_cmd[2] = 0;
    bms_cmd[3] = 255;
    SendCANData_BMS(TX_MAILBOX, 0x12D, bms_cmd, 8UL);

}
void BMS_Timeout_Handler(void)
{
    if(waitingResponse)
    {
        bmsTimeoutCounter++;

        if(bmsTimeoutCounter > BMS_TIMEOUT_LIMIT)
        {
            waitingResponse = false;
            bmsTimeoutCounter = 0;
        }
    }
}
void Log_Data(void)
{
	//float temp, float voltage, float speed

    sprintf(logBuffer,"%d.%d,%d,%d\r\n",variable/10,variable%10,(int)(DC_Current_Scaled),(int)vehicleSpeedKmph);
    UART_Send(logBuffer);
}
void FlexCAN_Callback_1(uint8_t instance,flexcan_event_type_t eventType,uint32_t buffIdx,flexcan_state_t *state)
{
(void)state;
(void)instance;
if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx ==TX_MAILBOX0))
{

}
if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx ==TX_MAILBOX))
{
g_canTx1Done = true;
bmsSendActive = false;
last_cmd_sent = bms_cmd_request;
}
if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx ==TX_MAILBOX2))
{

g_canTx3Done=true;
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == RX_MAILBOX5))
{
a++;
waitingResponse=true;
g_canRx0Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX5,&recvBuff306);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX6))
{
b++;
waitingResponse=true;
g_canRx2Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX6,&recvBuff308);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX7))
{
top++;
g_canRx1Done=true;
waitingResponse=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX7,&recvBuff307);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX8))
{
c++;
waitingResponse=true;
g_canRx3Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX8,&recvBuff309);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX9))
{
d++;
waitingResponse=true;
g_canRx4Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX9,&recvBuff310);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == RX_MAILBOX0))
{
e++;
waitingResponse=true;
g_canRx5Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX0,&recvBuff311);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX1))
{
f++;
g_canRx6Done=true;
waitingResponse=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX1,&recvBuff312);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX2))
{
g++;
g_canRx7Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX2,&recvBuff313);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX3))
{
g_canRx8Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX3,&recvBuff314);
}
if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx ==RX_MAILBOX4))
{
h++;
g_canRx9Done=true;
FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX4,&recvBuff315);
}
}
void FlexCAN_Callback_2(uint8_t instance,flexcan_event_type_t eventType,uint32_t buffIdx,flexcan_state_t *state)
{
(void)state;
(void)instance;
if ((eventType == FLEXCAN_EVENT_TX_COMPLETE) && (buffIdx ==TX_MAILBOX_IC_1))
{
g_canTx2Done=true;
}
}
void IC_CANTx(void)
{
IC_update();
split_odometer();
if(g_canTx2Done== true)
{
SendCANData_IC(TX_MAILBOX_IC_0, 0x62, IC_cmd0, 8UL);
SendCANData_IC(TX_MAILBOX_IC_1, 0x63, IC_cmd1, 8UL);
g_canTx2Done=false;
}
}
void Delay_Interlock(void)
{
	if(counter_flag==1)
    {
		count++;
	}
	if(count>50)
	{
		 PINS_DRV_SetPins(PTD,1<<8);
		 count=0;
		 counter_flag=0;
	}
}
void Delay_KeySwitch(void)
{
	if(Key_Switch_Counter_Flag==1)
	{
	Key_Switch_Counter++;
	}
	else
	{
    Key_Switch_Counter=0;
	}
	if(Key_Switch_Counter>800)
	{
		Key_Switch_Counter_Flag=0;

	    PINS_DRV_SetPins(PTB,1 << 2);

	}
}
void HV_Interlock_Check(void)
{
	Digital_Read_1=PINS_DRV_ReadPins(PTC)>>12 & 0x01;
	Digital_Read_3=PINS_DRV_ReadPins(PTE)>>15 & 0x01;
	if(	Digital_Read_1)
	{
	HV_Interlock=1;
	}
	else
	{
    HV_Interlock=0;
    bms_cmd_request=BMS_CMD_OFF;
	}
	if(Digital_Read_3)
	{
    HV_Interlock_2=1;
	}
	else
	{
	HV_Interlock_2=0;
	}
}
void rxCallback(void *driverState, uart_event_t event, void *userData)
{
    (void)driverState;
    (void)userData;

    if (event == UART_EVENT_RX_FULL)
    {
    dataReady = true;
    }

    LPUART_DRV_ReceiveData(INST_LPUART_1,buffer,255U);
}
uint8_t IsValidKnots(char *ptr, uint16_t len)
{
    uint8_t dotCount = 0;

    if (len < 3 || len > 7)
        return 0;

    for (uint16_t i = 0; i < len; i++)
    {
        if (ptr[i] == '.')
        {
            dotCount++;
        }
        else if (ptr[i] < '0' || ptr[i] > '9')
        {
            return 0;
        }
    }

    if (dotCount != 1)
        return 0;

    return 1;
}
void Capacity_Application(void)
{
SOC_Battery=(Capacity_Scaled*100)/165;
}
void SendSpeed(uint16_t kmph)
{
    char txBuf[6];
    uint8_t len = 0;
    if (kmph == 0)
    {
        txBuf[len++] = '0';
    }
    else
    {
        char rev[6];
        uint8_t i = 0;

        while (kmph > 0)
        {
            rev[i++] = (kmph % 10) + '0';
            kmph /= 10;
        }

        while (i > 0)
        {
            txBuf[len++] = rev[--i];
        }
    }
    txBuf[len++] = '\r';
    txBuf[len++] = '\n';

    LPUART_DRV_SendDataBlocking(INST_LPUART_1,(uint8_t *)txBuf,len,100);
}
void SendKnotsFloat(char *ptr, uint16_t len)
{
    char temp[16];
    char txBuf[32];
    for (uint16_t i = 0; i < len && i < 15; i++)
    temp[i] = ptr[i];
    temp[len] = '\0';
    float knots = atof(temp);
    int txLen = sprintf(txBuf, "%.3f\r\n", knots);
    LPUART_DRV_SendDataBlocking(INST_LPUART_1,(uint8_t *)txBuf,txLen,100);
}
uint16_t ConvertKnotsToKmph(char *ptr, uint16_t len)
{
    uint32_t knotsX10 = 0;
    uint8_t decimalSeen = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        if (ptr[i] == '.')
        {
            decimalSeen = 1;
        }
        else
        {
            knotsX10 = knotsX10 * 10 + (ptr[i] - '0');

            if (decimalSeen)
                break;
        }
    }

    if (!decimalSeen)
        knotsX10 *= 10;

    uint16_t kmph = (knotsX10 * 1852U) / 10000U;

    return kmph;
}
void CopyField(char *dest, char *src, uint16_t len)
{
   if (len > 15)
       len = 15;

   for (uint16_t i = 0; i < len; i++)
   {
       dest[i] = src[i];
   }

   dest[len] = '\0';
}
void SendKnots(char *ptr, uint16_t len)
{
    /* Send speed characters */
    LPUART_DRV_SendDataBlocking(INST_LPUART_1,(uint8_t*)ptr,len,100);
    uint8_t newline[2] = {'\r','\n'};
    LPUART_DRV_SendDataBlocking(INST_LPUART_1,newline,2,100);
}
void SendKnotsUART(char *ptr, uint16_t len)
{
    LPUART_DRV_SendDataBlocking(INST_LPUART_1,(uint8_t*)ptr,len,100);
    uint8_t newline[2] = {'\r','\n'};
    LPUART_DRV_SendDataBlocking(INST_LPUART_1,newline,2,100);
}
void ParseRMC(char *sentence)
{
    uint8_t field = 0;
    uint16_t i = 0;
    uint16_t start = 0;

    while (sentence[i] != '\0')
    {
        if (sentence[i] == ',' || sentence[i] == '*')
        {
            uint16_t len = i - start;

            if (field == 7)
            {
     if (IsValidKnots(&sentence[start], len))
                {
    	 comeback++;

                    uint16_t kmph = ConvertKnotsToKmph(&sentence[start], len);
                     vehicleSpeedKmph = kmph;

       //             SendKnotsUART(&sentence[start], len);

                }

                break;
            }
    field++;
    start = i + 1;
}
i++;
}
}
void ProcessSentence(char *sentence)
{
    if (sentence[0] == '$' &&
        sentence[1] == 'G' &&
		sentence[2] == 'N' &&
        sentence[3] == 'R' &&
        sentence[4] == 'M' &&
        sentence[5] == 'C')
    {
        ParseRMC(sentence);

    }
}
void ParseNMEA(uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        char ch = (char)data[i];

        /* Start of sentence */
        if (ch == '$')
        {
            nmeaIndex = 0;
        }

        if (nmeaIndex < (NMEA_MAX_LEN - 1))
        {
            nmeaSentence[nmeaIndex++] = ch;
        }

        /* End of sentence */
        if (ch == '\n')
        {
            nmeaSentence[nmeaIndex] = '\0';
            ProcessSentence(nmeaSentence);
            nmeaIndex = 0;
        }
    }
}
void Data_Logger_Function (void)
{

}
void Gear_State_Digital_Input(void)
{
static uint8_t prevSwapState = 0;
uint8_t currentSwapState = 0;
if(((PINS_DRV_ReadPins(PTC) >> 10) & 0x01) &&((PINS_DRV_ReadPins(PTC) >> 11) & 0x01) &&(!((PINS_DRV_ReadPins(PTC) >> 13) & 0x01)) &&(!((PINS_DRV_ReadPins(PTD) >> 17) & 0x01)))
{
currentSwapState = 1;
}
else
{
    currentSwapState = 0;
}
if((currentSwapState == 1) && (prevSwapState == 0)&&(vehicleSpeedKmph==0)&&(HV_Interlock))
{
	//HV_Interlock
    Blinq_Swap = 1;
}
prevSwapState = currentSwapState;
if(((PINS_DRV_ReadPins(PTC) >> 10) & 0x01)&&(!((PINS_DRV_ReadPins(PTC) >> 13) & 0x01))&&(!((PINS_DRV_ReadPins(PTC) >> 11) & 0x01))&&(!((PINS_DRV_ReadPins(PTD) >> 17) & 0x01)))
{
gear_state= DRIVE;
drive_status=D;
}
else
{
}
if(!((PINS_DRV_ReadPins(PTC) >> 10) & 0x01)&&(!((PINS_DRV_ReadPins(PTC) >> 13) & 0x01))&&(!((PINS_DRV_ReadPins(PTC) >> 11) & 0x01))&&(((PINS_DRV_ReadPins(PTD) >> 17) & 0x01)))
{
gear_state= NEUTRAL;
drive_status=N;
}
else
{
}
if(!((PINS_DRV_ReadPins(PTC) >> 10) & 0x01)&&(((PINS_DRV_ReadPins(PTC) >> 13) & 0x01))&&(!((PINS_DRV_ReadPins(PTC) >> 11) & 0x01))&&(!((PINS_DRV_ReadPins(PTD) >> 17) & 0x01)))
{
gear_state= REVERSE;
drive_status=R;
}
else
{

}
}
void BCM_Digital_input(void)
{
    static uint16_t acc_counter = 0;
    static uint16_t ign_counter = 0;

    static uint8_t acc_state = OFF;
    static uint8_t ign_state = OFF;
    uint8_t acc_pin = (PINS_DRV_ReadPins(PTD) >> 14) & 0x01;
    uint8_t ign_pin = (PINS_DRV_ReadPins(PTD) >> 13) & 0x01;
    if(acc_pin)
    {
        if(acc_counter< 1430)
            acc_counter++;
    }
    else
    {
        if(acc_counter > 0)
            acc_counter--;
    }
    if(acc_counter >= 1430)
    {
        acc_state = ON;
    }
    else if(acc_counter <= 5)
    {
        acc_state = OFF;
    }
    acc = acc_state;
    if(ign_pin)
    {
        if(ign_counter < 1430)
            ign_counter++;
    }
    else
    {
        if(ign_counter > 0)
            ign_counter--;
    }
    if(ign_counter >= 1430)
    {
        ign_state = ON;
    }
    else if(ign_counter <= 5)
    {
        ign_state = OFF;
    }
    ignition = ign_state;
    if(ignition == ON)
    {
        check = 1;
        car_status = IGNITION;
    }
    else if(acc == ON)
    {
        car_status = ACC;
    }
}
void Vehicle_Start_Sequence(void)
{
    if(acc == ON)
    {
   	bms_cmd_request=BMS_CMD_ON;
   	if(latch==1)
   	{
   	 Key_Switch_Counter_Flag=1;
   	 latch=0;
   	}
    }
    else
    {
    bms_cmd_request = BMS_CMD_OFF;
    }
    if((acc == ON) && (ignition == OFF))
    {
    gear_state = NEUTRAL;
    drive_status = N;
 	bms_cmd_request=BMS_CMD_ON;
    }
    if((acc == ON) && (ignition == ON))
    {
    if(latch_2==1)
    {
    counter_flag=1;
    latch_2=0;
    }
    if(gear_state == NEUTRAL)
    {
    drive_status = N;
    PINS_DRV_ClearPins(PTB,1 << 4);
    PINS_DRV_ClearPins(PTD,1 << 1);
    PINS_DRV_ClearPins(PTE, 1 << 9);
    }
    if(gear_state == DRIVE)
    {
    drive_status = D;
    PINS_DRV_ClearPins(PTB,1<< 4);
    delay(0x83D600);
    PINS_DRV_SetPins(PTD,1<<1);
    PINS_DRV_ClearPins(PTE, 1 << 9);
    }
    if(gear_state == REVERSE)
    {
     drive_status = R;
     PINS_DRV_ClearPins(PTD,1 <<1);
     delay(0x83D600);
     PINS_DRV_SetPins(PTB,1 << 4);
     PINS_DRV_SetPins(PTE, 1 << 9);
    }
    }
if((acc == OFF) && (ignition == OFF))
{
     PINS_DRV_ClearPins(PTD,1<<8);
	 PINS_DRV_ClearPins(PTB,1<<2);
	 latch_2=1;
	 latch=1;
    if(vehicleSpeedKmph > 0 && (gear_state == DRIVE))
    {
        bms_cmd_request=BMS_CMD_ON;
        drive_status = D;
        return;
    }
    if(vehicleSpeedKmph>0 &&(gear_state == REVERSE))
    {

    	bms_cmd_request=BMS_CMD_ON;
        drive_status = R;
         return;
    }
    if(vehicleSpeedKmph == 0)
    {
        drive_status =P;
        return;
     /*    gear_state = NEUTRAL;
         drive_status = N;
         bms_cmd_request=BMS_CMD_OFF;
         return;*/

    }

}
}
void BMS_Command_Manager(void)
{
    if(bms_cmd_request != last_cmd_sent)
    {
          bmsSendActive = true;
          g_canTx1Done = false;
    }
    if( (bmsSendActive) && (g_canTx1Done==false))
    {
        if(bms_cmd_request == BMS_CMD_ON)
        {
            BMS_Command_On();
        }
        else if(bms_cmd_request == BMS_CMD_OFF)
        {
            BMS_Command_Off();
        }
    }
    if(g_canTx1Done)
    {
        if(bms_cmd_request == BMS_CMD_ON)
        {
            if(charge_mos == 3)
            {
                bmsSendActive = false; // success
            }
            else
            {
                g_canTx1Done = false; // retry
            }
        }
        else if(bms_cmd_request == BMS_CMD_OFF)
        {
            if(charge_mos != 3)
            {
                bmsSendActive = false; // success
            }
            else
            {
                g_canTx1Done = false; // retry
            }
        }
    }
}
void BMS_Contactor_Check (void)
{

}

    int main(void)
    {
   	 Clock_init();
     GPIO_Init();
     Gear_Interrupt_init();
     FlexCANInit();
     FlexCANInit_IC();
     FLEXCAN_DRV_InstallEventCallback(INST_FLEXCAN_CONFIG_1,FlexCAN_Callback_1,NULL);
     FLEXCAN_DRV_InstallEventCallback(INST_FLEXCAN_CONFIG_2,FlexCAN_Callback_2,NULL);
     INT_SYS_EnableIRQ(CAN0_ORed_0_15_MB_IRQn);
     INT_SYS_EnableIRQ(CAN0_ORed_16_31_MB_IRQn);
     INT_SYS_EnableIRQ(CAN1_ORed_0_15_MB_IRQn);
 //  INT_SYS_EnableIRQ(CAN1_ORed_16_31_MB_IRQn);
     INT_SYS_EnableIRQGlobal();
	 RTC_Init();
	 PDB1_Trigger_Timer_Init();
	 ADC_Init();
	 if(!calculateIntValue(&pdb_config_1_timerConfig0, PDLY_TIMEOUT, &delayValue))
		{
	   while(1);
		}
	 PDB1_Init();
     DMA_Init();
     DMA_transfer(EDMA_CHN0_NUMBER, &(ADC1->R[0]),&ADC1_Sample, 3);
     TRGMUX_DRV_Init(INST_TRGMUX, &trgmux1_InitConfig0);
     LPIT_Init();
  /* CAN_ReceiveConfig1();
     CAN_ReceiveConfig2();
     CAN_ReceiveConfig3();
     CAN_ReceiveConfig4();*/
     CAN_ReceiveConfig6();
     CAN_ReceiveConfig7();
     CAN_ReceiveConfig8();
     CAN_ReceiveConfig9();
     CAN_ReceiveConfig10();
     CAN_ReceiveConfig11();
     CAN_ReceiveConfig12();
     CAN_ReceiveConfig13();
     CAN_ReceiveConfig14();
     CAN_ReceiveConfig15();
	 PINS_DRV_ClearPins(PTD,1<< 15);
     PINS_DRV_ClearPins(PTE,1<<9);
	 PINS_DRV_ClearPins(PTD,1<< 0);
     PINS_DRV_ClearPins(PTE,1<<8);
     PINS_DRV_ClearPins(PTB,1<<5);
	 PINS_DRV_ClearPins(PTC,1<<0);
	 PINS_DRV_ClearPins(PTD,1<<8);
	 PINS_DRV_ClearPins(PTB,1<<2);
//	 PINS_DRV_SetPins(PTD,1<<8);
	 LPUART_DRV_Init(INST_LPUART_1, &lpUartState0, &lpuart_0_InitConfig0);
	 LPUART_DRV_InstallRxCallback(INST_LPUART_1, rxCallback, NULL);
	 LPUART_DRV_ReceiveData(INST_LPUART_1,buffer,255U);
//	 PINS_DRV_SetPins(PTA,1<<16);
//	 PINS_DRV_ClearPins(PTA,1<<15);
     BMS_Req(TX_MAILBOX2,0x100,NULL,0U);
     FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX5,&recvBuff306);
     FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX7,&recvBuff307);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX6,&recvBuff308);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX8,&recvBuff309);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX9,&recvBuff310);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX0,&recvBuff311);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX1,&recvBuff312);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX2,&recvBuff313);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX3,&recvBuff314);
	 FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1,RX_MAILBOX4,&recvBuff315);
     for(;;)
     {
    /*	    count++;
    		if(count>500)
    		{
    			 PINS_DRV_SetPins(PTD,1<<8);
    			 count=0;
    			 counter_flag=0;
    		}*/
    /*	 if(comeback>100)
    	 {
    		 PINS_DRV_SetPins(PTE, 1 << 9);
    	 }*/
    	delay(0x6180);
    	HV_Interlock_Check();
    	Delay_Interlock();
    	Delay_KeySwitch();
    	Gear_State_Digital_Input();
    	BCM_Digital_input();
    	Vehicle_Start_Sequence();
    	if(log_data)
    	{
        Log_Data();
        log_data=0;
    	}
    	constant++;
		//25.5, 48.2, 1500
 /*   	if (DC_Current_Scaled > Current_Threshold || brake == ON)
    	{
    	braking_detected = 1;
    	}
    	else
    	{
    	braking_detected = 0;
    	}
    	a_est = K * DC_Current_Scaled;
    	v_est = v_prev + a_est * dt;
    	if (braking_detected)
    	{
    	    alpha = 0.8;
    	}
    	else
    	{
    	    alpha = 0.3;
    	}
    	display_speed = alpha * v_est + (1 - alpha) * gps_speed;
    	if (display_speed < 2)
    	display_speed = 0;               */
    	if (dataReady)
    	{
        dataReady = false;
    	ParseNMEA(buffer,255U);
    	}
        //  BMS_Request_Manager();
       //   BMS_Timeout_Handler();
      //    BMS_Contactor_Check ();
        IC_CANTx();
        BMS_Command_Manager();
    	Capacity_Application();
      if(exit_code != 0)
      {
      break;
      }
      }
        return exit_code;
      }
