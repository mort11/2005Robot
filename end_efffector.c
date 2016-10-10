/*******************************************************************************
* FILE NAME: end_effwector.c M.O.R.T. 2005 code
*
* DESCRIPTION:
* Controls all end effectors
*
* CHANGE LOG:
* 
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"
#include "mort2005.h"

int Grab_Status = 0;
int Grab_Timer = 0;

//This function moves the grabbing arm at a variable speed with limits at the top and bottom.  
//Arm can only be moved if the trgger is pressed 
void Service_end_effectors(void)
{
	GRAB_MOTOR = 127;
	GRAB_MOTOR = GRAB_JOYSTICK;
	
	if ( GRAB_BOTTOM_LIMIT == 0 && GRAB_JOYSTICK > 127 )
		GRAB_MOTOR = 127;

	if ( GRAB_TOP_LIMIT == 0 && GRAB_JOYSTICK < 127 )
		GRAB_MOTOR = 127;



//This function controls the extension and retraction of the hanging hook end effector
	if(HANG_HOOK_EXTEND == ON)
	{
		HANG_HOOK_FWD = ON;
		HANG_HOOK_REV = OFF;
	}

	if(HANG_HOOK_RETRACT == ON && HANG_BOTTOM_LIMIT == 1)
	{
		HANG_HOOK_FWD = OFF;
		HANG_HOOK_REV = ON;
	}

//This function controls the opening and closing of the ball grabbing end effector

	if(GRAB_TRIG_PULL == ON)
	{
		BALL_GRABBER_FWD = OFF;
		BALL_GRABBER_REV = ON;
	}

	if(BALL_GRABBER_CLOSE = ON)
	{
	BALL_GRABBER_FWD = ON;
	BALL_GRABBER_REV = OFF;
	}

	if((GRAB_TRIG_PULL == ON) && (Grab_Status == 0) && (Grab_Timer == 0))
	{
		BALL_GRABBER_FWD = OFF;
		BALL_GRABBER_REV = ON;
		Grab_Timer = 1;
	}

	if((GRAB_TRIG_PULL == OFF) && (Grab_Timer == 1) && (Grab_Status = 0))
	{
		Grab_Status = 1;
		Grab_Timer = 0;
	}

	if((GRAB_TRIG_PULL == ON) && (Grab_Status == 1) && (Grab_Timer == 0))
	{
   	    BALL_GRABBER_FWD = ON;
		BALL_GRABBER_REV = OFF;
		Grab_Timer = 1;
	}

	if((GRAB_TRIG_PULL == OFF) && (Grab_Timer == 1) && (Grab_Status = 1))
	{
		Grab_Status = 0;
		Grab_Timer = 0;
	}

}
