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


/* Including necessary configuration files. */
#include "sdk_project_config.h"

volatile int exit_code = 0;

/* User includes */
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
#define PDLY_TIMEOUT 	30UL
#define TX_MAILBOX  	(1UL)
#define TX_MSG_ID   	(1UL)
#define RX_MAILBOX0  	(0UL)
#define RX_MAILBOX1  	(2UL)
#define RX_MAILBOX2  	(3UL)
#define RX_MAILBOX3  	(4UL)
#define RX_MAILBOX4	(5UL)

#define TX_MAILBOX_IC_0  	(6UL)
#define TX_MAILBOX_IC_1  	(7UL)
#define RX_MSG_ID   		(2UL)

uint16_t delayValue = 0;
uint8_t mc_cmd[8];
uint8_t bms_cmd[8],	neutral_flag=0;
uint8_t DL_cmd0[8],DL_cmd1[8], DL_cmd2[8];
int neutral_count=0;
extern uint8_t IC_cmd0[8],IC_cmd1[8];
flexcan_msgbuff_t recvBuff301,recvBuff302,recvBuff303,recvBuff304,recvBuff305;
MYADCDMA0 ADC1_Sample;
void Clock_init(void)
{
	 /* Write your local variable definition here */
	 status_t status;

	/* Initialize clock module */
	status = CLOCK_DRV_Init(&clockMan1_InitConfig0);
	 DEV_ASSERT(status == STATUS_SUCCESS);

	 (void) status;

}


int main(void)
{
    /* Write your code here */
	Clock_init();

	GPIO_Init();

	Gear_Interrupt_init();

	FlexCANInit();

	FlexCANInit_IC();

	/**RTC timer is initialized with 15:43*/
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

	 CAN_ReceiveConfig1();

	 CAN_ReceiveConfig2();

	 CAN_ReceiveConfig3();

	 CAN_ReceiveConfig4();

	 CAN_ReceiveConfig5();

	//LPIT_Init();

    for(;;)
    {
    	delay(0x6A180);//according to 50ms
        printf("transmitted\n");
        IC_update();
        split_odometer();
		SendCANData_IC(TX_MAILBOX_IC_0, 0x62, IC_cmd0, 8UL);

		while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_2,TX_MAILBOX_IC_0)== STATUS_BUSY)
		 {
		    printf("wait for mailbox to be free\n");
		 }

		SendCANData_IC(TX_MAILBOX_IC_1, 0x63, IC_cmd1, 8UL);

		while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_2,TX_MAILBOX_IC_1)== STATUS_BUSY);

		SendCANData_IC(TX_MAILBOX_IC_1, 0x75, DL_cmd0, 8UL);

		while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_2,TX_MAILBOX_IC_1)== STATUS_BUSY);

		SendCANData_IC(TX_MAILBOX_IC_1, 0x74, DL_cmd1, 8UL);

		while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_2,TX_MAILBOX_IC_1)== STATUS_BUSY);

		SendCANData_IC(TX_MAILBOX_IC_1, 0x76, DL_cmd2, 8UL);

		while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_2,TX_MAILBOX_IC_1)== STATUS_BUSY);

		SendCANData(TX_MAILBOX, 0x101, mc_cmd, 8UL);
      
    	while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_1,TX_MAILBOX)== STATUS_BUSY);

//    	bms_cmd[0] = 0;
//    	bms_cmd[1] = 255;
//    	bms_cmd[2] = 0;
//    	bms_cmd[3] = 255;


    	bms_cmd[0] = 0;
    	bms_cmd[1] = 0;
    	bms_cmd[2] = 0;
    	bms_cmd[3] = 0;
		SendCANData_BMS(TX_MAILBOX, 0x12D, bms_cmd, 8UL);

    	while(FLEXCAN_DRV_GetTransferStatus(INST_FLEXCAN_CONFIG_1,TX_MAILBOX)== STATUS_BUSY);


    	FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1, RX_MAILBOX0, &recvBuff301);

		FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1, RX_MAILBOX1, &recvBuff302);

		FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1, RX_MAILBOX2, &recvBuff303);

		FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1, RX_MAILBOX3, &recvBuff304);

		FLEXCAN_DRV_Receive(INST_FLEXCAN_CONFIG_1, RX_MAILBOX4, &recvBuff305);

		DL_cmd0[3] = recvBuff303.data[0];
		DL_cmd0[4] = recvBuff303.data[1];
		DL_cmd0[5] = recvBuff303.data[2];
		DL_cmd0[6] = recvBuff303.data[3];

		DL_cmd2[0] = recvBuff302.data[6];
		DL_cmd2[1] = recvBuff302.data[7];
		DL_cmd2[2] = recvBuff302.data[4];
		DL_cmd2[3] = recvBuff302.data[5];
		DL_cmd2[4] = recvBuff301.data[6];
		DL_cmd2[5] = recvBuff301.data[7];
		DL_cmd2[6] = recvBuff303.data[6];
		DL_cmd2[7] = recvBuff303.data[7];

        if(exit_code != 0)
        {
            break;
        }
    }
    return exit_code;
}


/* END main */
/*!
** @}
*/
