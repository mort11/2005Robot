/*******************************************************************************
* FILE NAME: demo.c
*
* DESCRIPTION:
* This file contains autonomous mode functions
*
*******************************************************************************/
#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_camera.h"
#include "MORT2005.H"

void go_for_red_shirt(void);


/*************************************************************************************
                                   MORT VARIABLES                                  
*************************************************************************************/
extern int tracking;
extern int pan_servo_copy;
extern int tilt_servo_copy;
extern int tracking_state;
static int old_x, old_y;
static int pan_servo;


// Defines for panning the camera to look for Tetras or Goals
		#define SWEEP_STEP			 5
		#define CENTER_SWEEP_MIN	 97
		#define CENTER_SWEEP_MAX  	157


// Defines for states of the Autonomous State Machine looking for vision tetra
		#define STOP_AND_PAN			1
		#define	CHECK_FOR_TRACKING		2
		#define PIVOT_ROBOT 			3
		#define	GO_TO_TARGET			4
		#define	WAIT_FOR_MOVEMENT		5


// This is the minimum speed applied to the wheels to pivot the robot until the camera pan servo is within the PIVOT_WINDOW.
// Too large a value will cause robot to pivot faster than the pan servo can react. Too small a value will cause robot never
// reach the center position. This value is directly related and should be tuned according to the camera's COARSE_TRACKING values for pivoting.
		#define PIVOT_SPEED_RIGHT		127-22
		#define PIVOT_SPEED_LEFT		127+22
		#define PIVOT_MULTIPLIER		1


// This is the speed at which we start to move towards the target after pivoting
		#define FORWARD_SPEED		 	20
		#define FORWARD_SPEED_MIN		10
		#define FORWARD_SPEED_MULTIPLIER 1

// This corrects the robot steering when the camera's pan servo changes from looking forward. The camera pan servo value is multiplied
// by this factor and then applied to the wheels. The bigger the multiplier, the move radical the robot corrects, or SWERVES.
// This value is directly related and should be tuned according to the camera's FINE_TRACKING setting.
		#define	SWERVE_MULTIPLIER		1


		#define DEMO_TILT_SERVO_AT_SHIRT	110   


//********************************************************************************************************
//
//
//							AUTONOMOUS MODE STATE MACHINE
//
//		
//********************************************************************************************************

void go_for_red_shirt(void)
	{
	int x, y;

	switch( tracking_state ) {
	//******************************************************************************************************
	// Ignore all activity if state = 0;
	case 0:
	break;

	//******************************************************************************************************
	// First lets make sure we're stopped and cycle camera until we find a target
	case STOP_AND_PAN:
	   	p1_x = 127;													// Stop wheels
   		p1_y = 127;

		// Make sure both servos are set to COARSE_TRACKING
		if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
			stop_streaming( );										// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);	// Pan.servo = COARSE, Tilt still COARSE
			restart_streaming( );									// Start resending TC packets
		}

		// Now sweep the camera's pan servo position
		printf( "Sweeping for Someone's shirt..." );   
		pan_servo -= SWEEP_STEP; 									// step to center
		if(pan_servo < CENTER_SWEEP_MIN)							// if past max position
			pan_servo = CENTER_SWEEP_MAX;							// reset to left start position
		printf("%d\r", pan_servo);									// Print our current camera pan servo position
		stop_streaming( );											// Stop sending TC packets ( streaming )
		camera_set_servos( pan_servo, 128 ); 						// position camera to look for tetra 
		restart_streaming( );										// Start resending TC packets
		printf("Positioning pan.servo to %d....Entering CHECK_FOR_TRACKING\r", pan_servo);				
		tracking_state = CHECK_FOR_TRACKING;						// go to next state
		break;

	//******************************************************************************************************
	// Check if we have aquired a target, IF SO THEN pivot TOWARDS target
	case CHECK_FOR_TRACKING:
	   	p1_x = 127;												// Stop wheels
   		p1_y = 127;												// Stop wheels
		if( tracking == 0 )										// If camera lost tracking
			tracking_state = STOP_AND_PAN;						// try to aquire again !!!
		else {													// WE are tracking
			printf("Target Found...PIVOTING robot...Entering PIVOT_ROBOT\r");
			tracking_state = PIVOT_ROBOT;						// Go to next state
		}	
	break;	

	//******************************************************************************************************
	// PIVOT robot until camera is looking straight
	case PIVOT_ROBOT:
		if( tracking == 0 ) { 									// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");// print it
			tracking_state = STOP_AND_PAN;						// try to aquire again
			break;
		}				
		// GO_TO_TARGET when really close to target center
		if( (pan_servo_copy >= 123) && (pan_servo_copy <= 131) ) {// If camera is looking forward
			printf("PIVOTING done....Target centered...Entering GO_TO_TARGET\r"); // Print it
			tracking_state = GO_TO_TARGET;						// Starts towards target
			break;
		}
	

			p1_y = 127;											// Don't allow Forward speed
			x = 127;											// start off at zero turning
			
			// Lets PIVOT faster if we're really crooked and slow down as we straighten out
			// This crazy math keeps p1_x from underflowing below overflowing above 255
			if( pan_servo_copy > 127 ) {						// If we're swerving left
				x -= ((pan_servo_copy - 127) * PIVOT_MULTIPLIER);// apply multiplier to pivot right
				if( x < 0 ) x = 0;								// Never let x go below zero
				if( x > PIVOT_SPEED_RIGHT)						// If slower than minimum speed
					x = PIVOT_SPEED_RIGHT;						// set speed to minimum speed
				if( old_x != x)									// If we have a change in speed
					printf("Pivoting RIGHT %d\r", x);			// Print it out
			}
			// This crazy math keeps p1_x from underflowing below zero
			if( pan_servo_copy < 127 ) {						// If we're swerving right
				x += ((127 - pan_servo_copy) * PIVOT_MULTIPLIER);// apply multiplier to pivot right
				if( x > 255 ) x = 255;							// never let x go over 255
				if( x < PIVOT_SPEED_LEFT)						// if slower that minimum speed
					x = PIVOT_SPEED_LEFT;						// set speed to minimum
				if( old_x != x)									// if we have a change in speed
					printf("Pivoting LEFT %d\r", x);			// Print it out
			}
			old_x = x;											// save so we can detect a change next loop
			p1_x = x;											// set p1_x to x
			break;
		


	//******************************************************************************************************
	// Now that we are looking straight (hopefully)
	case GO_TO_TARGET:
		if( tracking == 0 ) { 										// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// Print it
			tracking_state = STOP_AND_PAN;							// try to aquire again
			break;
		}				

		// Check if robot is really close to target..If so then re-check centering
		if(tilt_servo_copy < DEMO_TILT_SERVO_AT_SHIRT) {			// If close enough to target
			printf("Robot @ shirt...STOPPING\r"); // print it
			tracking_state = WAIT_FOR_MOVEMENT;						// Now stop and wait
			break;
		}

		y = 127 + FORWARD_SPEED;									// Head forwards towards target								
		x = 127;													// Assume we're going to go straight

		// Slow down robot the closer we get to the target by using tilt servo count and multiplier
		y -= ((127 - tilt_servo_copy) * FORWARD_SPEED_MULTIPLIER);
		if (y < (127 + FORWARD_SPEED_MIN))
			y = 127 + FORWARD_SPEED_MIN;		

		// This crazy math keeps p1_x from overflowing above 255
		if( pan_servo_copy > 127 ) {								// If we're swerving right
			x -= ((pan_servo_copy - 127) * SWERVE_MULTIPLIER);		// apply multiplier to swerve left
			if( x < 0 ) x = 0;										// Never let x go below zero
			if( old_x != x)											// If we have a change in speed
				printf("Swerving RIGHT...Speed = %d\r", x);			// Print it out
		}															// END if( pan_servo_copy > 127)
		// This crazy math keeps p1_x from underflowing below zero
		if( pan_servo_copy < 127 ) {								// If we're swerving left
			x += ((127 - pan_servo_copy) * SWERVE_MULTIPLIER);		// swerve left
			if( x > 255 ) x = 255;									// never let x go over 255
			if( old_x != x)											// if we have a change in speed
				printf("Swerving LEFT....Speed = %d\r", x);			// Print it out
		}															// END if( pan_servo_copy < 127)
		old_x = x;													// save so we can detect a change next loop
		p1_x = x;													// set p1_x to x
		p1_y = y;													// set p1_y to y

		break;

	//******************************************************************************************************
	case WAIT_FOR_MOVEMENT:
	   	p1_x = 127;													// Stop wheels
  		p1_y = 127;													// Stop wheels
		// Check if shirt has moved away
		if(tilt_servo_copy > DEMO_TILT_SERVO_AT_SHIRT + 5) {
			printf("Target moved...need to go FORWARD\r");
			tracking_state = GO_TO_TARGET;
		}

		if( (pan_servo_copy <= 122) || (pan_servo_copy >= 132)) {
			printf("Target moved...need to PIVOT.....\r"); // Print it
			tracking_state = PIVOT_ROBOT;
		}
		break;


	}  // END switch( tracking_state )				
}

