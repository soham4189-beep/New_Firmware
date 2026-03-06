/*
 * rtc.c
 *
 *  Created on: 24-Jul-2025
 *      Author: Rohit
 */

#include "sdk_project_config.h"
#include "rtc.h"

#define PBRIDGE_RTC_SLOT    29

rtc_timedate_t Current_Time;

void swSupervisorMode(void)
{
    __asm(" MOVS R0, #0x0 ");
    __asm(" MSR CONTROL, R0 ");
    __asm("DSB");
    __asm("ISB");
}

/* Switch to user mode */
void swUserMode(void)
{
    __asm(" MOVS R0, #0x1 ");
    __asm(" MSR CONTROL, R0 ");
    __asm("DSB");
    __asm("ISB");
}

void SVC_Handler(void)
{
    /* Switch to supervisor mode need to be done through an exception handler*/
    swSupervisorMode();
}

void RTC_Init()
{
	uint8_t regIdx = PBRIDGE_RTC_SLOT/8;
	  RTC_DRV_Init(RTC_1, &rtc_1InitCfg0);

	  /* Set APIS to allow usermode access to RTC Memory Space */
	      /* Clear bit position to grant usermode access level */
	      AIPS->OPACR[regIdx] &= ~AIPS_OPACR_SP5_MASK;
	      /* Switch to user mode */
	      swUserMode();

	  /* Set the time and date */
		 RTC_DRV_SetTimeDate(RTC_1, &rtc_1InitCfg0_StartTime);

		 /* Start the RTC counter */
		 RTC_DRV_StartCounter(RTC_1);

		 /* Switch to supervisor mode by calling exception SVC_Handler */
		 __asm("svc #0x32");

}
