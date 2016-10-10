/*******************************************************************************
* FILE NAME: autonomous.c
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
#include "autonomous.h"
#include "MORT2005.H"

/*************************************************************************************
                                   MORT VARIABLES                                  
*************************************************************************************/
extern int tracking;						// Flag that is set when sucessfully tracking a color
extern int our_delay;
extern int pan_servo_copy;
extern int tilt_servo_copy;
extern int auton_mode;
extern int vision_sweep;
extern int goal_sweep;

int tracking_state;							// state of Autonomous State machine
int color = 0;								// Keeps track of current color we're looking for
int	got_tetra = 0;
int old_x, old_y;
int pan_servo;

// This function determines which autonomous mode is done
int get_mode(void){
	int mode = 0;
		if(MODE_SWITCH_A0 == 0) // Checks the dipswitch position for MODE_SWITCH_A0 
			mode += 1;
		if(MODE_SWITCH_A1 == 0) // Checks the dipswitch position for MODE_SWITCH_A1
			mode += 2;
		if(MODE_SWITCH_A2 == 0) // Checks the dipswitch position for MODE_SWITCH_A2
			mode += 4;
return mode;
}

// This function determines which way we sweep for vision tetra
int get_vision_sweep(void){
	int vision_sweep = 0;
		if(VISION_SWEEP_A0 == 0) // Checks the dipswitch position for VISION_SWEEP_A0 
			vision_sweep += 1;
		if(VISION_SWEEP_A1 == 0) // Checks the dipswitch position for VISION_SWEEP_A1
			vision_sweep += 2;
return vision_sweep;
}

// This function determine which way we sweep when looking for a goal to cap
int get_goal_sweep(void){
	int goal_sweep = 0;
		if(GOAL_SWEEP_A0 == 0) // Checks the dipswitch position for GOAL_SWEEP_A0 
			goal_sweep += 1;
		if(GOAL_SWEEP_A1 == 0) // Checks the dipswitch position for GOAL_SWEEP_A1
			goal_sweep += 2;
return goal_sweep;
}


void MORT_autonomous_mode( int mode )
{

// PMR hard code for now because we don't have dipswitches
mode = GO_FOR_VISION_TETRA;
vision_sweep = 3;
goal_sweep = 3; 								// If we are sweeeping about the center

	switch(mode) {

	case AUTONOMOUS_LOAD:
		if(tracking_state == 0) {						// If this is the first time 
			printf("Autonomous Mode = Autonomous Load\r");
			tracking_state = LOADING_STILL;				// Startup autonomous state machine
		}
		load_goal( );									// Run the State Machine
		break;

	case GO_FOR_VISION_TETRA:
		if( tracking_state == 0) {						// If this is the 1st time
			printf("Autonomous Mode = Go For Vision Tetra\r");
			tracking_state = STOP_AND_PAN;				// Startup the state machine
			color = GREEN;								// Set GREEN as the color to track
			camera_find_color(color);					// Start tracking GREEN
			if (vision_sweep == 1)						// If we are sweeping LEFT to RIGHT
				pan_servo = LEFT_SWEEP_START;			// Start panning from the left
			if (vision_sweep == 2)						// If we are sweeping RIGHT to LEFT
				pan_servo = RIGHT_SWEEP_START;			// Start panning from the right
			if (vision_sweep == 3)						// If we are sweeeping about the center
				pan_servo = CENTER_SWEEP_START;			// Start panning from center
		}
		go_for_vision_tetra( );							// Run the State Machine
		break;

	default:
		break;
	}
}



//********************************************************************************************************
//
//
//							AUTONOMOUS MODE STATE MACHINE
//
//		
//********************************************************************************************************
void go_for_vision_tetra(void)
	{
	int x, y;

	switch( tracking_state ) {
	//******************************************************************************************************
	// Ignore all activity if state = 0;
	case 0:
	break;


	case GO_LOAD_HEIGHT:
		set_tower_position(TETRA_LOAD_HEIGHT);
		our_delay = 19;
		tracking_state = STOP_AND_PAN;
		break;

	case GO_LOAD_DELAY:
		set_tower_position(TETRA_LOAD_HEIGHT);
		if(our_delay--)
			tracking_state = STOP_AND_PAN;
		break;

	//******************************************************************************************************
	// First lets make sure we're stopped and cycle camera until we find a target
	case STOP_AND_PAN:
		set_tower_position(TETRA_LOAD_HEIGHT);
	   	p1_x = 127;												// Stop wheels
   		p1_y = 127;												// Stop wheels
		// Make sure both servos are DISABLED
		if(get_pan_tracking( ) != DISABLE && get_tilt_tracking( ) != DISABLE) {
			stop_streaming( );									// Stop sending TC packets ( streaming )
			camera_auto_servo(DISABLE, DISABLE);				// Pan.servo = COARSE, Tilt still COARSE
			restart_streaming( );								// Start resending TC packets
		}
		// Now sweep the camer's pan servo position
		printf( "Sweeping for TETRA..." );   
		if (vision_sweep == 1) {								// If we are sweeping LEFT to RIGHT
			printf( "Left  " );   								// print it
			pan_servo -= SWEEP_STEP; 							// step to center
			if(pan_servo < LEFT_SWEEP_MAX)						// if past max position
				pan_servo = LEFT_SWEEP_START;					// reset to left start position
		}
		if (vision_sweep == 2) {								// If we are sweeping RIGHT to LEFT
			printf( "Right  " );   								//  print it
			pan_servo += SWEEP_STEP;							// step to center
			if(pan_servo > RIGHT_SWEEP_MAX)						// if past max position
				pan_servo = RIGHT_SWEEP_START;					// reset to right start position
		} 
		if (vision_sweep == 3) {								// If we are sweeping about the CENTER
			printf( "Center  " );  								// print it
			pan_servo -= SWEEP_STEP; 							// step to center
			if(pan_servo < CENTER_SWEEP_MAX)					// if past max position
				pan_servo = CENTER_SWEEP_START;					// reset to left start position
		}
		printf("%d\r", pan_servo);								// Print our current camera pan servo position
		stop_streaming( );										// Stop sending TC packets ( streaming )
		camera_set_servos( pan_servo, 150 ); 					// position camera to look for tetra 
		restart_streaming( );									// Start resending TC packets
		printf("Positioning pan.servo to %d....Entering CHECK_FOR_TRACKING\r", pan_servo);				
		tracking_state = CHECK_FOR_TRACKING;					// go to next state
		break;

	//******************************************************************************************************
	// Check if we have aquired a target, IF SO THEN pivot TOWARDS target
	case CHECK_FOR_TRACKING:
		set_tower_position(TETRA_LOAD_HEIGHT);
	   	p1_x = 127;												// Stop wheels
   		p1_y = 127;												// Stop wheels
		if( tracking == 0 )										// If camera lost tracking
			tracking_state = STOP_AND_PAN;						// try to aquire again !!!
		else {													// WE are tracking
			printf("Target Found...PIVOTING robot...Entering PIVOT_ROBOT\r");
			stop_streaming( );									// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, DISABLE);		// Pan.servo = COARSE, Tilt,servo = DISABLED
			restart_streaming( );								// Start resending TC packets
			tracking_state = PIVOT_ROBOT;						// Go to next state
		}	
	break;	

	//******************************************************************************************************
	// PIVOT robot until camera is looking straight
	case PIVOT_ROBOT:
		set_tower_position(TETRA_LOAD_HEIGHT);
		if( tracking == 0 ) { 									// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");// print it
			tracking_state = STOP_AND_PAN;						// try to aquire again
			break;
		}				
		// GO_TO_TARGET when really close to target center
		if( (pan_servo_copy >= 126) && (pan_servo_copy <= 128) ) {// If camera is looking forward
			printf("PIVOTING done....Target centered...Entering GO_TO_TARGET\r"); // Print it
			// Make sure both servos are set to coarse_tracking
			if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
				stop_streaming( );										// Stop sending TC packets ( streaming )
				camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);	// Set both servos to coarse
				restart_streaming( );									// Start resending TC packets
			}
			tracking_state = GO_TO_TARGET;						// Starts towards target
			break;
		}
		// Pivot robot towards target
		p1_y = 127;												// Don't allow Forward speed
		p1_x = 0;
		
		if( pan_servo_copy == 127 ) 							// If we're swerving left
				p1_x = 127;
		if( pan_servo_copy < 127 ) 							// If we're swerving left
				p1_x = PIVOT_SPEED_RIGHT;							// set speed to minimum speed
		if( pan_servo_copy > 127 ) 							// If we're swerving right
				p1_x = PIVOT_SPEED_LEFT;							// set speed to minimum
		break;

	//******************************************************************************************************
	// Now that we are looking straight (hopefully)
	case GO_TO_TARGET:
		set_tower_position(TETRA_LOAD_HEIGHT);
		if( tracking == 0 ) { 										// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// Print it
			tracking_state = STOP_AND_PAN;							// try to aquire again
			break;
		}				
		// Check if we are out of control by comparing cam pan servo with max swerve
		if( pan_servo_copy > 180 || pan_servo_copy < 75 ) {			// if too far off course
			printf("*** ABORT**** At MAX_SWERVE....Entering  STOP_AND_PAN\r");
			tracking_state = STOP_AND_PAN;							// Try to aquire again
			break;
		}
		// Ok camera is still within MAX swerve, now adjust course if necessary
		y = 127 + FORWARD_SPEED;									// Head forwards towards target								
		x = 127;													// Assume we're going to go straight

		// Check if we're pointing down enough to start slowing down
		if (tilt_servo_copy > TILT_SERVO_FINE_SWITCH) {				// If pointing down enough
			if(get_tilt_tracking( ) != FINE_TRACKING ) {			// Make sure PAN servo if FINE_TRACKING
				stop_streaming( );									// Stop sending TC packets ( streaming )
				camera_auto_servo(COARSE_TRACKING, FINE_TRACKING);	// Pan and Tilt = FINE Tracking
				restart_streaming( );								// Restart sending TC packets
			}														// END if(get_tilt_tracking( ) != FINE_TRACKING)
			// Slow down robot the closer we get to the target by using tilt servo count and multiplier
			y -= ((tilt_servo_copy - 127) * FORWARD_SPEED_MULTIPLIER); // slow down faster if furtehr away
			if (y < (127 + FORWARD_SPEED_MIN))						// if slower than minimum
				y = 127 + FORWARD_SPEED_MIN;						// output minimum speed
		}															// END if(tilt_servo_copy > TILT_SERVO_FINE_SWITCH)
		// This crazy math keeps p1_x from overflowing above 255
		if( pan_servo_copy < 127 ) {								// If we're swerving right
			x -= ((127 - pan_servo_copy) * SWERVE_MULTIPLIER);		// apply multiplier to swerve left
			if( x < 0 ) x = 0;										// Never let x go below zero
			if( old_x != x)											// If we have a change in speed
				printf("Swerving RIGHT...Speed = %d\r", x);			// Print it out
		}															// END if( pan_servo_copy > 127)
		// This crazy math keeps p1_x from underflowing below zero
		if( pan_servo_copy > 127 ) {								// If we're swerving left
			x += ((pan_servo_copy + 127) * SWERVE_MULTIPLIER);		// swerve left
			if( x > 255 ) x = 255;									// never let x go over 255
			if( old_x != x)											// if we have a change in speed
				printf("Swerving LEFT....Speed = %d\r", x);			// Print it out
		}															// END if( pan_servo_copy < 127)
		old_x = x;													// save so we can detect a change next loop
		p1_x = x;													// set p1_x to x
		p1_y = y;													// set p1_y to y

		// Check if robot is really close to target..If so then re-check centering
		if(tilt_servo_copy > TILT_SERVO_BEFORE_POSITION) {			// If close enough to target
			printf("Robot @ TILT_SERVO_BEFORE_POSITION...STOPPING to Recheck Centering...Entering STOP_SHORT\r"); // print it
			our_delay = 19;											// Delay before final check
			tracking_state = STOP_SHORT;							// Now stop and wait
		}
		break;

	//******************************************************************************************************
	case STOP_SHORT:
		set_tower_position(TETRA_LOAD_HEIGHT);
   		p1_x = 127;														// Stop wheels
  		p1_y = 127;														// Stop wheels
	if(our_delay-- == 0) {											// if delay is done
		printf("Delay is done...Entering FINAL_PIVOT\r");			// Print it out
		tracking_state = FINAL_PIVOT;								// check if centered
	}
	break;

	//******************************************************************************************************
	// First state pivots robot until camera is looking straight
	case FINAL_PIVOT:
		set_tower_position(TETRA_LOAD_HEIGHT);
		if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != FINE_TRACKING) {
			stop_streaming( );										// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, FINE_TRACKING);		// Both servos to FINE_TRACKING
			restart_streaming( );									// Start resending TC packets
		}
		if( tracking == 0 ) { 										// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// print it
			tracking_state = STOP_AND_PAN;							// try to aquire again
			break;
		}				
		if( (pan_servo_copy >= 126) && (pan_servo_copy <= 128) ) {	// If camera is centered
			printf("Camera Centered..Entering  HIT_TARGET\r");		// Print it out
			tracking_state = STRAIGHT_TO_TARGET;					// Starts towards target
		}
		// Pivot robot towards target
		p1_y = 127;													// Don't allow Forward speed
		p1_x = 0;
		if( pan_servo_copy == 127 ) 							// If we're swerving left
				p1_x = 127;
		if( pan_servo_copy < 127 ) 							// If we're swerving left
				p1_x = PIVOT_SPEED_RIGHT;							// set speed to minimum speed
		if( pan_servo_copy > 127 ) 							// If we're swerving right
				p1_x = PIVOT_SPEED_LEFT;							// set speed to minimum
		break;

	//******************************************************************************************************
	case STRAIGHT_TO_TARGET:
		set_tower_position(TETRA_LOAD_HEIGHT);
		if( tracking == 0 ) { 										// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// Print it
			tracking_state = STOP_AND_PAN;							// try to aquire again
			break;
		}				
		p1_y = 127 + FORWARD_SPEED_MIN;								// Head forwards towards target								
		p1_x = 127;													// Assume we're going to go straight

		if(tilt_servo_copy > TILT_SERVO_AT_TETRA) {				// if close enough to target
			printf("Robot @ TILT_SERVO_AT_TETRA...Entering AT_TETRA\r"); // print it
			tracking_state = AT_TETRA;								// Go to next state
		}
		break;

	//******************************************************************************************************
	// Now we are at the target
	case AT_TETRA:
		set_tower_position(TETRA_UP_HEIGHT);
	   	p1_x = 127;													// Stop wheels
	  	p1_y = 127;													// Stop wheels
		printf("Lifting ARM....Entering WATCH_IT_RISE\r");			// print it	
break;
		got_tetra = 0;												// clear got_tetra flag
		our_delay = 38;
		tracking_state = 55;
		break;

	//******************************************************************************************************
	case 55:
		set_tower_position(TETRA_UP_HEIGHT);
	   	p1_x = 127;														// Stop wheels
   		p1_y = 127;														// Stop wheels
		if(our_delay-- == 0) {											// if delay is done
			tracking_state = GOT_TETRA_UP;								// check if centered
		}
		break;

	//******************************************************************************************************
	case WATCH_IT_RISE:
		set_tower_position(TETRA_UP_HEIGHT);
		if(tilt_servo_copy < TILT_SERVO_TETRA_UP )					// if the tilt servo is higher than the TILT_SERVO_TETRA up position
			got_tetra = 1;											// set the tetra_up flag
		if ( TOWER_TOP_LIMIT == 0 ) { 								// if ARM up sensor blocked
			if( got_tetra == 1) {									// if we saw the tetra rise
				printf("Got Vision Tetra\r");						// print it
				tracking_state = GOT_TETRA_UP;						// Success...now find the goal
			}														
			// SHIT!!!  We missed the tetra... Start the recovery code
			else {													// Did not see the tetra
				printf("Missed it...Backing up to TILT_SERVO_BEFORE_POSITION\r"); // print it
				stop_streaming( );									// Stop sending TC packets ( streaming )
				camera_auto_servo(FINE_TRACKING, FINE_TRACKING);	// Pan.servo = COARSE, Tilt still COARSE
				restart_streaming( );								// Start resending TC packets
				set_tower_position(TETRA_LOAD_HEIGHT);
				tracking_state = WAIT_FOR_BACKUP;					// startup the recovery code
			}
		}															// END if (TOWER_TOP_LIMIT == 0)
		break;

	//******************************************************************************************************
	//   WE MISSED THE TETRA.....BEGIN BACKUP and RECOVERY CODE
	//******************************************************************************************************
	case WAIT_FOR_BACKUP:
		set_tower_position(TETRA_LOAD_HEIGHT);
		p1_x = 127;													// stop left and right motion
		p1_y = REVERSE_SPEED_MIN;									// stop forward motion
		if(tilt_servo_copy < TILT_SERVO_BEFORE_POSITION) {			// if we backed up far enough
			TOWER_MOTOR = TOWER_DOWN_SPEED;							// Lower ARM
			printf("Lowering ARM\r");								// print it
			our_delay = 38;											// ARM down time
			tracking_state = ARM_DOWN_DELAY;						// go to next state
		}
		break;

	//******************************************************************************************************
	case ARM_DOWN_DELAY:
		set_tower_position(TETRA_LOAD_HEIGHT);
	   	p1_x = 127;													// stop all motion
   		p1_y = 127;													// stop all motion
		TOWER_MOTOR = TOWER_DOWN_SPEED;								// Arm down speed
      	if (our_delay-- == 0) {										// If arm down delay is done			
			printf("ARM is down...Trying again by Entering FINAL_PIVOT\r");	
			tracking_state = FINAL_PIVOT;
		}
		break;
	//******************************************************************************************************
	//   END OF BACKUP and RECOVERY CODE
	//******************************************************************************************************


	//******************************************************************************************************
	case GOT_TETRA_UP:
		set_tower_position(TETRA_UP_HEIGHT);
		p1_x = 127;											// Stop wheels
   		p1_y = 127;											// Stop wheels
		if (goal_sweep == 1) 								// If we are sweeping LEFT to RIGHT
			pan_servo = LEFT_SWEEP_START;					// Start panning from the left
		if (goal_sweep == 2) 								// If we are sweeping RIGHT to LEFT
			pan_servo = RIGHT_SWEEP_START;					// Start panning from the right
		if (goal_sweep == 3) 								// If we are sweeeping about the center
			pan_servo = CENTER_SWEEP_START;					// Start panning from center
		camera_find_color (YELLOW);							// Start tracking for YELLOW !!!! Must change to YELLOW !!!
		tracking_state = FIND_GOAL;							// Start looking for GOAL
		break;

	//******************************************************************************************************
	case FIND_GOAL:
		set_tower_position(TETRA_UP_HEIGHT);
		p1_x = 127;													// Stop wheels
   		p1_y = 127;													// Stop wheels
		if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
			stop_streaming( );										// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);	// Pan.servo = COARSE, Tilt Disable
			restart_streaming( );									// Start resending TC packets
		}
		printf( "Sweeping for GOAL..." );   
		if( goal_sweep == 1 ) {
			printf( "Left  " );   
			pan_servo -= SWEEP_STEP; 
			if(pan_servo < LEFT_SWEEP_MAX)
				pan_servo = LEFT_SWEEP_START;
		}
		if( goal_sweep == 2 ) {
			printf( "Right  " );   
			pan_servo += SWEEP_STEP;
			if(pan_servo > RIGHT_SWEEP_MAX)
				pan_servo = RIGHT_SWEEP_START;
		} 
		if( goal_sweep == 3 ) {
			printf( "Center  " );   
			pan_servo += SWEEP_STEP;
			if(pan_servo > CENTER_SWEEP_MAX)
				pan_servo = CENTER_SWEEP_START;
		} 
		printf("%d\r", pan_servo);								// Print our current camera pan servo position
		stop_streaming( );										// Stop sending TC packets ( streaming )
		camera_set_servos( pan_servo, 150 ); 					// position camera to look for tetra 
		restart_streaming( );									// Start resending TC packets
		printf("Positioning pan.servo to %d....Entering CHECK_FOR_GOAL_TRACKING\r", pan_servo);				
		tracking_state = CHECK_FOR_GOAL_TRACKING;				// go to next state
		break;


	//******************************************************************************************************
	case CHECK_FOR_GOAL_TRACKING:
		set_tower_position(TETRA_UP_HEIGHT);
	   	p1_x = 127;												// Stop wheels
   		p1_y = 127;												// Stop wheels
		if( tracking == 0 )										// If camera lost tracking
			tracking_state = FIND_GOAL;							// try to aquire again !!!
		else {													// WE are tracking
			printf("GOAL Found...PIVOTING robot...Entering PIVOT_TO_GOAL\r");
			stop_streaming( );									// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);// Pan.servo = COARSE, Tilt,servo = DISABLED
			restart_streaming( );								// Start resending TC packets
			tracking_state = PIVOT_TO_GOAL;						// Go to next state
		}	
	break;	


	//******************************************************************************************************
	case PIVOT_TO_GOAL:
		set_tower_position(TETRA_UP_HEIGHT);
		if( tracking == 0 ) { 									// If we lost tracking
			printf("Lost Tracking....Entering  FIND_GOAL\r");
			tracking_state = FIND_GOAL;						    // try to aquire again
			break;
		}				

		// GO_TO_TARGET when really close to target center
		if( (pan_servo_copy > 126) && (pan_servo_copy < 128) ) {	// If camera is looking forward
			if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
				stop_streaming( );									// Stop sending TC packets ( streaming )
				camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);// Pan.servo = COARSE, Tilt still COARSE
				restart_streaming( );								// Start resending TC packets
			}
			printf("Pivot done...Going to GOAL\r");					// Print it
			tracking_state = GO_TO_GOAL;							// Starts towards target
			break;
		}
		// Pivot robot towards target
		p1_x = 127;
		p1_y = 127;													// Don't allow Forward speed
		if( pan_servo_copy < 127 ) 									// If we're swerving left
				p1_x = PIVOT_SPEED_RIGHT;							// set speed to minimum speed
		if( pan_servo_copy > 127 ) 									// If we're swerving right
				p1_x = PIVOT_SPEED_LEFT;							// set speed to minimum
		break;


	//******************************************************************************************************
	case GO_TO_GOAL:
		set_tower_position(TETRA_UP_HEIGHT);
		if( tracking == 0 ) { 											// If we lost tracking
			printf("Lost Tracking....Entering  FIND_GOAL\r");			// Print it
			tracking_state = FIND_GOAL; 								// try to aquire again
			break;
		}			
		// Check if we are out of control by comparing cam pan servo with max swerve
		if( pan_servo_copy > 180 || pan_servo_copy < 75 ) {				// if too far off course
			printf("*** ABORT**** At MAX_SWERVE....Entering  FIND_GOAL\r");
			tracking_state = FIND_GOAL;									// Try to aquire again
			break;
		}
		// Ok camera is still within MAX swerve, now adjust course if necessary
		y = 127 + FORWARD_SPEED;										// Head forwards towards target								
		x = 127;														// Assume we're going to go straight

		// Check if we're pointing down enough to start slowing down
		if (tilt_servo_copy > TILT_SERVO_FINE_GOAL_SWITCH) {			// If pointing down enough
			if(get_tilt_tracking( ) != FINE_TRACKING ) {				// Make sure PAN servo if FINE_TRACKING
				stop_streaming( );										// Stop sending TC packets ( streaming )
				camera_auto_servo(COARSE_TRACKING, FINE_TRACKING);		// Pan and Tilt = FINE Tracking
				restart_streaming( );									// Restart sending TC packets
			}
			// Slow down robot the closer we get to the target by using tilt servo count and multiplier
			y -= ((tilt_servo_copy - 127) * FORWARD_SPEED_MULTIPLIER);
			if (y < (127 + FORWARD_SPEED_MIN))
				y = 127 + FORWARD_SPEED_MIN;
		}
		
		// This crazy math keeps p1_x from overflowing above 255
		if( pan_servo_copy < 127 ) {										// If we're swerving right
			x -= ((127 - pan_servo_copy) * SWERVE_MULTIPLIER);			// apply multiplier to swerve left
			if( x < 0 ) x = 0;											// Never let x go below zero
			if( old_x != x)												// If we have a change in speed
				printf("Swerving RIGHT %d\r", x);						// Print it out
		}
		// This crazy math keeps p1_x from underflowing below zero
		if( pan_servo_copy > 127 ) {										// If we're swerving left
			x += ((pan_servo_copy + 127) * SWERVE_MULTIPLIER);			// swerve left
			if( x > 255 ) x = 255;										// never let x go over 255
			if( old_x != x)												// if we have a change in speed
				printf("Swerving LEFT %d\r", x);						// Print it out
		}
		old_x = x;														// save so we can detect a change next loop
		p1_x = x;														// set p1_x to x
		p1_y = y;														// set p1_y to y

		if(tilt_servo_copy > TILT_SERVO_AT_GOAL) {					// if close enough to target
			printf("AT GOAL....Entering AT_GOAL\r");						// print it
			stop_streaming( );											// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);		// Pan and Tilt = COARSE Tracking
			restart_streaming( );										// Restart sending TC packets
			tracking_state = AT_GOAL;									// what is next ?
		}
		break;

	break;


	//******************************************************************************************************
	case AT_GOAL:
		set_tower_position(TETRA_UP_HEIGHT);
		p1_x = 127;														// stop all motion
	   	p1_y = 127;														// stop all motion
		printf("Lowering ARM....Entering CAP_IT\r");	
		our_delay = 38;							// arm down time
		tracking_state = FINAL_GOAL_PIVOT;		// go to next state
	break;


	//******************************************************************************************************
	// First state pivots robot until camera is looking straight
	case FINAL_GOAL_PIVOT:
		set_tower_position(TETRA_UP_HEIGHT);
		if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != FINE_TRACKING) {
			stop_streaming( );										// Stop sending TC packets ( streaming )
			camera_auto_servo(COARSE_TRACKING, FINE_TRACKING);		// Both servos to FINE_TRACKING
			restart_streaming( );									// Start resending TC packets
		}
		if( tracking == 0 ) { 										// If we lost tracking
			printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// print it
			tracking_state = STOP_AND_PAN;							// try to aquire again
			break;
		}				
		if( (pan_servo_copy >= 126) && (pan_servo_copy <= 128) ) {	// If camera is centered
			printf("Camera Centered..Entering  HIT_TARGET\r");		// Print it out
			tracking_state = GO_GOAL_HEIGHT;					// Starts towards target
		}
		// Pivot robot towards target
		p1_y = 127;													// Don't allow Forward speed
		p1_x = 0;
		if( pan_servo_copy == 127 ) 							// If we're swerving left
				p1_x = 127;
		if( pan_servo_copy < 127 ) 							// If we're swerving left
				p1_x = PIVOT_SPEED_RIGHT;							// set speed to minimum speed
		if( pan_servo_copy > 127 ) 							// If we're swerving right
				p1_x = PIVOT_SPEED_LEFT;							// set speed to minimum
		break;


	//******************************************************************************************************
	case GO_GOAL_HEIGHT:
		set_tower_position(GOAL_UP_HEIGHT);
		our_delay = 76;
		tracking_state = GO_GOAL_DELAY;
		break;


	//******************************************************************************************************
	case GO_GOAL_DELAY:
		set_tower_position(GOAL_UP_HEIGHT);
		if(our_delay--)
			tracking_state = GO_A_LITTLE_FORWARD;
		break;


	//******************************************************************************************************
	case GO_A_LITTLE_FORWARD:
		set_tower_position(GOAL_UP_HEIGHT);
		p1_x = 127;
		p1_y = 127 + FORWARD_SPEED_MIN;
		our_delay = 19;
		tracking_state = A_LITTLE_DELAY;
		break;


	//******************************************************************************************************
	case A_LITTLE_DELAY:
		set_tower_position(GOAL_UP_HEIGHT);
		if( our_delay--) {
			tracking_state = CAP_IT; 
		}
		break;


	//******************************************************************************************************
	case CAP_IT:
		set_tower_position(GOAL_DOWN_HEIGHT);
	   	p1_x = 127;									// stop all motion
   		p1_y = 127;									// stop all motion
		our_delay = 19;
		tracking_state = CAP_IT_DELAY;
	break;


	//******************************************************************************************************
	case CAP_IT_DELAY:
		set_tower_position(GOAL_DOWN_HEIGHT);
		if( our_delay--)
			tracking_state = GO_A_LITTLE_REVERSE;
		break;


	//******************************************************************************************************
	case GO_A_LITTLE_REVERSE:
		set_tower_position(GOAL_DOWN_HEIGHT);
		p1_x = 127;
		p1_y = 127 - REVERSE_SPEED_MIN;
		our_delay = 19;
		tracking_state = REVERSE_DELAY;
		break;


	//******************************************************************************************************
	case REVERSE_DELAY:
		set_tower_position(GOAL_DOWN_HEIGHT);
		if( our_delay-- ) 
			tracking_state = AUTONOMOUS_DONE;
		break;


	//******************************************************************************************************
	case AUTONOMOUS_DONE:
		set_tower_position(0);
		p1_x = 127;
		p1_y = 127;
		TOWER_MOTOR = 127;							// Arm down speed
	break;

	}  // END switch( tracking_state )				
}





void load_goal(void)
{

	// Second robot autonomous mode

	switch (tracking_state)  {

	case 0:
	break;

	case LOADING_STILL:
		p1_x = 127;
		p1_y = 127;
		our_delay = 2;
		if(our_delay-- == 0) {											// if delay is done
			printf("LOADING_STILL done...Entering LOADING_FORWARD\r");							// Print it out
			tracking_state == LOADING_FORWARD;
		}
		our_delay = 10;	
		break;
	
	case LOADING_FORWARD:
		p1_x = 127;
		p1_y = 157;
		if(our_delay-- == 0) {											// if delay is done
			printf("LOADING_FORWARD done...Entering LOADING_FORWARD_DELAY\r");		// Print it out
			tracking_state == LOADING_FORWARD_DELAY;
		}
		our_delay = 60;
		break;

	case LOADING_FORWARD_DELAY:
		p1_x = 127;
		p1_y = 157;
		if(our_delay-- == 0) {											// if delay is done
			printf("LOADING_FORWARD_DELAY done...Entering LOADING_TURN_LEFT\r");			// Print it out
			tracking_state == LOADING_TURN_LEFT;
		}
		our_delay = 20;
		break;

	case LOADING_TURN_LEFT:
		p1_x = 37;
		p1_y = 127;
		if(our_delay-- == 0) {											// if delay is done
			printf("LOADING_TURN_LEFT done...Entering FINAL_APPROACH\r");			// Print it out
			tracking_state == LOADING_FINAL_APPROACH;
		}
		our_delay = 60;
		break;

	case LOADING_FINAL_APPROACH:
		p1_x = 127;
		p1_y = 157;
		if(our_delay-- == 0) {											// if delay is done
			printf("FINAL_APPROACH done...Entering LOADING_AT_TARGET\r");			// Print it out
			tracking_state == LOADING_AT_TARGET;
		}
		our_delay = 10;
		break;

	case LOADING_AT_TARGET:
		p1_x = 127;
		p1_y = 127;
		if(our_delay-- == 0) {											// if delay is done
			printf("LOADING_AT_TARGET done... At Final Goal\r");						// Print it out
		}
		break;
	}
}
	





