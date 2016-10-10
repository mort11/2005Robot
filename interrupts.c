/*******************************************************************************
*
*	TITLE:		interrupts.c 
*
*	VERSION:	0.2 (Beta)                           
*
*	COMMENTS:	This file contains template interrupt and timer
*				initialization/handling code for the IFI EDU and
*				FRC robot controllers.
*
*				Numbers within brackets refer to the PIC18F8520
*				data sheet page number where more information can
*				be found.
*
*               This file best viewed with tabs set to four.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2003  0.1  RKW Original
*	25-Feb-2004  0.2  RKW - Added the ability to clear the interrupt flag before
*	                  enabling the interrupt.
*
*******************************************************************************/

#include "ifi_picdefs.h"
#include "interrupts.h"
#include "ifi_aliases.h"
#include "MORT2005.h"

int tower_count, arm_count;


/*******************************************************************************
*
*	FUNCTION:		Initialize_Interrupts()
*
*	PURPOSE:		Initializes the interrupt hardware.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Place "#include "interrupts.h" in the includes section
*					of user_routines.c then call Initialize_Interrupts() in
*					user_routines.c/User_Initialization().
*
*******************************************************************************/

void Initialize_Interrupts(void)  
{
	// initialize external interrupt 1 (INT2 on user 18F8520)
	TRISBbits.TRISB2 = 1;		// make sure the RB2/INT2 pin is configured as an input [108]
								//
	INTCON3bits.INT2IP = 0;		// 0: interrupt 1 is low priority (leave at 0 for IFI controllers) [91]
								// 1: interrupt 1 is high priority
								//
	INTCON2bits.INTEDG2 = 0;	// 0: trigger on the falling-edge [90]
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT2IF = 0;		// 0: external interrupt 1 hasn't happened (set to 0 before enabling the interrupt) [91]
								// 1: external interrupt 1 has happened
								//
	INTCON3bits.INT2IE = 1;		// 0: disable interrupt	1 [91]
								// 1: enable interrupt 1
	
	// initialize external interrupt 2 (INT3 on user 18F8520)
	TRISBbits.TRISB3 = 1;		// make sure the RB3/CCP2/INT3 pin is configured as an input [108]
								//
	INTCON2bits.INT3IP = 0;		// 0: interrupt 2 is low priority (leave at 0 for IFI controllers) [90]
								// 1: interrupt 2 is high priority
								//
	INTCON2bits.INTEDG3 = 0;	// 0: trigger on the falling-edge [90]
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT3IF = 0;		// 0: external interrupt 2 hasn't happened (set to 0 before enabling the interrupt) [91]
								// 1: external interrupt 2 has happened
								//
	INTCON3bits.INT3IE = 1;		// 0: disable interrupt	2 [91]
								// 1: enable interrupt 2
}
						  

/*******************************************************************************
*
*	FUNCTION:		Int_1_Handler()
*
*	PURPOSE:		If enabled, the interrupt 1 handler is called when the
*					interrupt 1/digital input 1 pin changes logic level. The 
*					edge that the interrupt 1 pin reacts to is programmable 
*					(see comments in the Initialize_Interrupts() function, 
*					above).
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The PIC18F8520's RB2/INT2 pin on port b is mapped to
*					interrupt 1 on the EDU-RC and digital input 1 on the
*					FRC-RC [108].
*
*******************************************************************************/
void Int_1_Handler(void)
{
}

/*******************************************************************************
*
*	FUNCTION:		Int_2_Handler()
*
*	PURPOSE:		If enabled, the interrupt 2 handler is called when the
*					interrupt 2/digital input 2 pin changes logic level. The 
*					edge that the interrupt 2 pin reacts to is programmable 
*					(see comments in the Initialize_Interrupts() function, 
*					above).
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The PIC18F8520's RB3/CCP2/INT3 pin on port b is mapped
*					to interrupt 2 on the EDU-RC and digital input 2 on the
*					FRC-RC [108].
*
*******************************************************************************/
void Int_2_Handler(void)
{
	// this function will be called when an interrupt 2 occurs
	// this function will be called when an interrupt 1 occurs
	if(TOWER_CHANNEL_B == 0)
		tower_count++;
	else
		tower_count--;
	if(TOWER_HOME_SWITCH == 0)
		tower_count = 0;
}

