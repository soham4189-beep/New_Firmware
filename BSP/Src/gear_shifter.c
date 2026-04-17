/*
 * gear_shifter.c
 *
 *  Created on: 17-Jul-2025
 *      Author: Rohit
 */
#include "sdk_project_config.h"
#include "gear_shifter.h"
#include "gpio.h"
uint32_t int_status;
uint8_t Swap_Flag=0,Drive_Flag=0,Debounce_Check_Flag=0,drive_state_change_Flag=0,Gear_Shifter_DebounceCheck=0;
extern volatile uint8_t gear_state,Debounce_Tick;
extern volatile uint8_t drive_status,neutral_flag,Drive_Pin_Status,Swap_Pin_Status;

void Gear_Interrupt_init()
{
	/* Setup button pins interrupt
	* Set interrupt for rising edge for PE14*/
 //   PINS_DRV_SetPinIntSel(BTN_PORTE, BTN1_PIN, PORT_INT_FALLING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORTB, BTN2_PIN, PORT_INT_FALLING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORTB, BTN3_PIN, PORT_INT_FALLING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORTB, BTN4_PIN, PORT_INT_FALLING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORTB, BTN5_PIN, PORT_INT_FALLING_EDGE);
	PINS_DRV_SetPinIntSel(PORTD,13, PORT_INT_FALLING_EDGE);
	PINS_DRV_SetPinIntSel(PORTD,14, PORT_INT_FALLING_EDGE);
//	PINS_DRV_SetPinIntSel(PORTE,15,PORT_INT_FALLING_EDGE);
//	PINS_DRV_SetPinIntSel(PORTE,16,PORT_INT_FALLING_EDGE);
	PINS_DRV_SetPinIntSel(PORTC,13, PORT_INT_FALLING_EDGE);
//	PINS_DRV_SetPinIntSel(PORTC,11, PORT_INT_FALLING_EDGE);
//	PINS_DRV_SetPinIntSel(PORTC,10, PORT_INT_FALLING_EDGE);
	/* Install buttons ISR for portE */
//	INT_SYS_InstallHandler(PORTE_IRQn, &PortE_ISR, NULL);//install interrupt hdlr for porte
	/* Install buttons ISR for portB */
//	INT_SYS_InstallHandler(PORTB_IRQn, &PortB_ISR, NULL);//install interrupt hdlr for portb
//	INT_SYS_InstallHandler(PORTC_IRQn, &PortC_ISR, NULL);//install interrupt hdlr for portb
	/* Enable buttons interrupt */
//	INT_SYS_EnableIRQ(PORTE_IRQn);//enable interrupt for porte
	/* Enable buttons interrupt */
//	INT_SYS_EnableIRQ(PORTB_IRQn);//enable interrupt for portb
//	INT_SYS_EnableIRQ(PORTC_IRQn);//enable interrupt for portb
}
void Debounce_Check(void)
{
Drive_Pin_Status=PINS_DRV_ReadPins(PTC)>>13 & 0x01;
Swap_Pin_Status=PINS_DRV_ReadPins(PTC)>>11 & 0x01;
}
void PortE_ISR()
{

}
void PortB_ISR()
{
	printf("portB ISR invoked\n");
	int_status = PINS_DRV_GetPortIntFlag(BTN_PORTB);
	if(int_status & (1 << BTN2_PIN) )
	{
		gear_state= DRIVE;
	}
	else if(int_status & (1 << BTN3_PIN))
	{
	if(((int_status & (1 << BTN2_PIN)) ==0) && ((int_status & (1 << BTN4_PIN)) ==0)\
				&& ((int_status & (1 << BTN5_PIN))==0) )
	{
		gear_state= NEUTRAL;

	}
	}
	else if(int_status & (1 << BTN4_PIN))
	{

	}
	else if(int_status & (1 << BTN5_PIN))
	{
		gear_state= REVERSE;
	}
//	Gear_Shifter_DebounceCheck=1;
	int_status &= ~(0xFFFFFFFF);
	PINS_DRV_ClearPortIntFlagCmd(BTN_PORTB);
	drive_state_change_Flag=1;
    drive_state_change();
}
/*
 void PortB_ISR(void)
{
    uint32_t flags;

    flags = PINS_DRV_GetPortIntFlag(BTN_PORTB);

    if ((flags & (1UL << BTN2_PIN)) != 0UL)
    {
        GearFsm_RequestEvent(GEAR_EVT_REQ_DRIVE);
    }
    else if ((flags & (1UL << BTN3_PIN)) != 0UL)
    {
        GearFsm_RequestEvent(GEAR_EVT_REQ_NEUTRAL);
    }
    else if ((flags & (1UL << BTN5_PIN)) != 0UL)
    {
        GearFsm_RequestEvent(GEAR_EVT_REQ_REVERSE);
    }
    else
    {

    }

    PINS_DRV_ClearPortIntFlagCmd(BTN_PORTB);
}
*/
void PortC_ISR(void)
{
    int_status = PINS_DRV_GetPortIntFlag(PORTC);
    Debounce_Check_Flag=1;
	int_status &= ~(0xFFFFFFFF);
    PINS_DRV_ClearPortIntFlagCmd(PORTC);
}
