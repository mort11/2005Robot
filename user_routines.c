/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/
#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include "user_camera.h"
#include "MORT2005.H"
#include "autonomous.h"


/*************************************************************************************
                                   MORT VARIABLES                                  
*************************************************************************************/
unsigned char 	dual_joysticks = 1;					// Set to 1 for dual joystick control
unsigned char 	Accelleration  = 0;					// Set to non ZERO to enable accell limiting, smaller value will slow accell/decell.


int 			_100millisecond_timer = 0;			// Used to count how many 26.2 millisecond intervals till 100 milliseconds
int 			_1second_timer = 0;					// Used to count how many 26.2 millisecond intervals till 1 second 
int 			p1aux1_debounce = 0;				// Aux1 button debounce
int 			p1aux2_debounce = 0;				// Aux2 button debounce
int 			tracking = 0;						// Flag that is set when sucessfully tracking a color
int	  			right_wheel_save = 127;				// Saves last RIGHT wheel value for ACCELL limit
int				left_wheel_save  = 127;				// Saves last LEFT  wheel value for ACCELL limit
int				fine_tracking = 0;					// Set to 1 to allow fine trackin, but SLOW
int				mode_switch = 0;					// Autonomous mode selector switch
int 			our_delay;
int				pan_servo_copy, tilt_servo_copy;
int 			tracking_debounce = 0;				
int 			last_destination = 0;
int				tower_difference = 0;
int				auton_mode 		 = 0;
int				vision_sweep	 = 0;
int				goal_sweep 		 = 0;
int				test_counter	 = 0;
int				last_tower_count = 0;


extern 	int				tracking_state;
extern 					cam_struct 		cam;
extern unsigned char 	aBreakerWasTripped;	
extern int 				tower_count, arm_count;


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{

 Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  	Serial_Driver_Initialize();

	Initialize_Interrupts();
  	
	auton_mode = get_mode( );
	vision_sweep = get_vision_sweep( );
	goal_sweep = get_goal_sweep( );

  	Putdata(&txdata);             /* DO NOT CHANGE! */

	User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
    Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
	
	service_tower( );


	// If camera was initialized and is ready, then process camera info
	if(Camera_Initialized( )) {

//		Uncomment this code to perform camera exposure test ****************************************************************
//		find_exposure( GREEN );
//		Putdata(&txdata);
//		return;	

		// Check if Camera has found the color
	  	if (camera_track_update( )) {					// If we received a packet from camera
			if (cam.count >= CAMERA_MINIMUM_PIXELS)	{ 
				if (tracking_debounce++ >= 3){
					tracking = 1;
					tracking_debounce = 3;
					pan_servo_copy  = cam.pan_servo;
					tilt_servo_copy = cam.tilt_servo; 
				}			
			}
			else {
				if( tracking_debounce-- <= 0){
					tracking_debounce = 0;
					tracking = 0;
				}
			}
		} // END if(camera_track_update()
	} // END if(Camera_Initialized()



	if(dual_joysticks == 1)										// If using dual joysticks	
	{
		LEFT_WHEEL =  255 - p1_y;								// Simply output p1_y to LEFT wheel
		RIGHT_WHEEL = p2_y;										// Simply output p2_y to RIGHT wheel
	}
	else														// else combine X-Y to single joystick drive
	{
		p1_x = 255-p1_x;										// Reverse p1_x direction
	   	LEFT_WHEEL  = 255 - (Limit_Mix(2000 + p1_y + p1_x - 127));
		RIGHT_WHEEL = Limit_Mix(2000 + p1_y - p1_x + 127);
	}

	if( Accelleration != 0 )							// If accelleration limiting is required
		SoftStartWheels( );								// Prevent rapid changes @ wheels to reduce slippage

	
	if( _100millisecond_timer++ >= 19) {
		_100millisecond_timer = 0;
//		printf("Autonomous Mode = %d  Vision Sweep = %d  Goal Sweep = %d\r", auton_mode, vision_sweep, goal_sweep);
//		printf("P3_Y = %d   P3_X = %d   P4_Y = %d   P4_X = %d\r", p3_y, p3_x, p4_y, p4_x);
//		printf("Tower_count = %04d  Tower_Motor = %03d\r", tower_count, TOWER_MOTOR);
//		printf("Tower Joystick = %d\r", TOWER_JOYSTICK);

	}		
	if( _1second_timer++ >= 38) {
		_1second_timer = 0;
		printf("Tower_count = %04d  Tower_Motor = %03d\r", tower_count, TOWER_MOTOR);

	//Do stuff here every second
		
	}

	Update_IO( );										// Update Robot feedback LEDS on OI

  	Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

	Putdata(&txdata);             						// DO NOT CHANGE!
}


/*******************************************************************************
* FUNCTION NAME: Update_OI
* PURPOSE:       Updates the Operator Interface LEDS and displays Robot status
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Update_IO( void )
{
 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
	int i;

  	if (user_display_mode == 0) /* User Mode is Off */
	{
		Pwm1_red = Pwm1_green = 0;      		// Turn PWM1 LEDs OFF
	    
		if (p1_x >= 125 && p1_x <= 129)
	    	Pwm1_red  = 1;      				// Turn ON PWM1 red LED
	    if (p1_y >= 125 && p1_y <= 129)
	      	Pwm1_green  = 1;      				// Turn ON PWM1 green LED
	  
 	}
	else  										// User Mode is On - displays data in OI 4-digit display*/
  	{
    	User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  	}   

	/* Example code to check if a breaker was ever tripped. */
	if (aBreakerWasTripped) {
		for (i=1;i<29;i++) {
	    	if (Breaker_Tripped(i))
	      		User_Byte1 = i;  // Update the last breaker tripped on User_Byte1 (to demonstrate the use of a user byte)  
	                          	 //	Normally, you do something else if a breaker got tripped (ex: limit a PWM output)
	  	}
	}
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2001)
  {
    limited_value = 2001;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*============================================================================
Limit wheel pwm accelleration and decelleration of robot by ramping up and down to target
velocity. Prevents wheel slippage, reduces current peaks and saves wear and tear on drive
train. If accelleration limiting is NOT needed, set the variable accelleration = 0 to disable

NOTE: 	This routine must be called AFTER the combine_joysticks( ) and compensate_jowstick( )
		routines. It should be called just before the output function.
=============================================================================*/
void SoftStartWheels( void )
{
	// Check for accell limit
	if (LEFT_WHEEL > left_wheel_save) 						// If we are accellerating
	{
		if (LEFT_WHEEL > (left_wheel_save+Accelleration)) 	// If we must limit accell
			left_wheel_save+=Accelleration;					// Step to next higher speed
		else
			left_wheel_save = LEFT_WHEEL;
	}

	// Check for decell limit
	if (LEFT_WHEEL < left_wheel_save) 						// If we are decellerating
	{
		if (LEFT_WHEEL < (left_wheel_save-Accelleration)) 	// If we must limit decell
			left_wheel_save-=Accelleration;					// Step to next lower speed
		else
			left_wheel_save = LEFT_WHEEL;
	}
	

	// Check for accell limit
	if (RIGHT_WHEEL > right_wheel_save) 						// If we are accellerating
	{
		if (RIGHT_WHEEL > (right_wheel_save+Accelleration)) 	// If we must limit accell
			right_wheel_save+=Accelleration;					// Step to next higher speed
		else
			right_wheel_save = RIGHT_WHEEL;
	}

	// Check for decell limit
	if (RIGHT_WHEEL < right_wheel_save) 						// If we are decellerating
	{
		if (RIGHT_WHEEL < (right_wheel_save-Accelleration)) 	// If we must limit decell
			right_wheel_save-=Accelleration;					// Step to next lower speed
		else
			right_wheel_save = RIGHT_WHEEL;
	}
	
	LEFT_WHEEL  = left_wheel_save;
	RIGHT_WHEEL = right_wheel_save;
}


void service_tower( void )
{
//	TOWER_JOYSTICK += 33;

	 if(TOWER_JOYSTICK > 117 && TOWER_JOYSTICK < 137 ) {
		if( last_tower_count == 0 )
			last_tower_count = tower_count;
		set_tower_position(last_tower_count * IPP);
	}

	if(TOWER_JOYSTICK > 127 )  {
		last_tower_count = 0;
		if(TOWER_JOYSTICK > (127 + TOWER_UP_SPEED))
			TOWER_JOYSTICK = 127 + TOWER_UP_SPEED;
	}

	if(TOWER_JOYSTICK < 127 ) {
		last_tower_count = 0;
		if(TOWER_JOYSTICK < (127 - TOWER_DOWN_SPEED))
		TOWER_JOYSTICK = 127 - TOWER_DOWN_SPEED;
	}

	if (TOWER_HOME_SWITCH == 0 && TOWER_JOYSTICK < 127)
		TOWER_JOYSTICK = 127;
	
	if (TOWER_TOP_LIMIT == 0 && TOWER_JOYSTICK > 127)
		TOWER_JOYSTICK = 127;

	TOWER_MOTOR = TOWER_JOYSTICK;
}			


void set_tower_position(int destination)
{

	destination = destination/IPP;			  		// Convert INCHES to counts. IPP is .0523 inches/pulse

	if( tower_count > destination) {
		TOWER_MOTOR = 127;
		return;
	}

	tower_difference = destination - tower_count;

	if (tower_difference > TOWER_UP_SPEED)
		TOWER_MOTOR = 127 + TOWER_UP_SPEED;
	
	else
		TOWER_MOTOR = 127 + tower_difference;
}


