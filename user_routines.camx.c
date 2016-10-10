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


/*************************************************************************************
                                   MORT VARIABLES                                  
*************************************************************************************/
unsigned char 	dual_joysticks = 0;					// Set to 1 for dual joystick control
unsigned char 	Accelleration  = 0;					// Set to non ZERO to enable accell limiting, smaller value will slow accell/decell.
int 			_100millisecond_timer = 0;			// Used to count how many 26.2 millisecond intervals till 100 milliseconds
int 			_1second_timer = 0;					// Used to count how many 26.2 millisecond intervals till 1 second 
int 			p1aux1_debounce = 0;				// Aux1 button debounce
int 			p1aux2_debounce = 0;				// Aux2 button debounce
int 			pan_dir = 0;						// Keeps track of current pan ( LEFT/CENTER/RIGHT );
int 			color = 0;							// Keeps track of current color we're looking for
int 			tracking = 0;						// Flag that is set when sucessfully tracking a color
int	  			right_wheel_save = 127;				// Saves last RIGHT wheel value for ACCELL limit
int				left_wheel_save  = 127;				// Saves last LEFT  wheel value for ACCELL limit
int 			tracking_state = 0;
int				fine_tracking = 0;					// Set to 1 to allow fine trackin, but SLOW
int				mode_switch = 0;					// Autonomous mode selector switch
int 			our_delay, x, old_x, y, old_y;
int 			pan_servo;
int 			vision_sweep_mode;
int 			goal_sweep_mode;
int 			tracking_debounce = 0;
int				got_tetra;

#define LEFT 	0
#define RIGHT	1
#define CENTER	2


#define LEFT_SWEEP_START  	180
#define LEFT_SWEEP_MAX  	100

#define RIGHT_SWEEP_START  	80
#define RIGHT_SWEEP_MAX  	160

#define CENTER_SWEEP_START  100
#define CENTER_SWEEP_MAX  	150

#define SWEEP_STEP			5

#define HANG_UP_SPEED 		90
#define HANG_UP_HOLD_SPEED 	110
#define HANG_DOWN_SPEED 	135


/****************************
**   VISION VARIABLES      **
*****************************/
extern cam_struct 		cam;
extern unsigned int 	index_ptr, data_rdy, parse_mode;
extern unsigned char 	aBreakerWasTripped;
extern int calibrate_running;

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

  Putdata(&txdata);             /* DO NOT CHANGE! */

  Serial_Driver_Initialize();
  printf("IFI 2005 User Processor Initialized ...\r");  /* Optional - Print initialization message. */
  /* Note:  use a '\r' rather than a '\n' with the new compiler (v2.4) */

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
	
  	Getdata(&rxdata);   								// Get fresh data from the master microprocessor.

				// Test stuff for 2004 arm
				HANG_MOTOR = 127;

				// < 127 = LOWER   > 127 = RAISE
				if(HANG_JOYSTICK < HANG_UP_SPEED)
					HANG_JOYSTICK = HANG_UP_SPEED;
			
				if(HANG_JOYSTICK > HANG_DOWN_SPEED)
					HANG_JOYSTICK = HANG_DOWN_SPEED;
			
				HANG_MOTOR = HANG_JOYSTICK;
				
				if ( HANG_BOTTOM_LIMIT == 0 && HANG_JOYSTICK > 127 )
					HANG_MOTOR = 127;
			
				if ( HANG_TOP_LIMIT == 0 && HANG_JOYSTICK < 127 )
					HANG_MOTOR = HANG_UP_HOLD_SPEED;
			

	mode_switch = read_selector_switch( ); 				// Read Autonomous mode selector switch and save

	// Turn ON Compressor if needed
  	COMPRESSOR_FWD = !COMPRESSOR_SWITCH;// Power pump only if pressure switch is off
  	COMPRESSOR_REV = 0;

	// Do stuff here every 100 milliseconds
	if( _100millisecond_timer++ >= 4) {
		_100millisecond_timer = 0;

      	if( tracking ) {
//			printf(" @ X=%03d Y=%03d Count=%03d Conf=%03d Pan=%03d Tilt=%03d LEFT=%03d RIGHT=%03d\r",cam.x,cam.y,cam.count,cam.conf,cam.pan_servo,cam.tilt_servo, LEFT_WHEEL, RIGHT_WHEEL );
			if(color == RED) 	Relay1_green ^= 1;
			if(color == GREEN) 	Relay2_green ^= 1;
			if(color == BLUE) 	Switch1_LED  ^= 1;
			if(color == YELLOW) Switch2_LED  ^= 1;
			if(color == WHITE) 	Switch3_LED  ^= 1;
		}
		else {
	 	 	Relay1_green=0; Relay2_green=0; Switch1_LED=0; Switch2_LED=0; Switch3_LED=0; 
			if(color == RED) 	Relay1_green = 1;
			if(color == GREEN)  Relay2_green = 1;
			if(color == BLUE) 	Switch1_LED  = 1;
			if(color == YELLOW) Switch2_LED  = 1;
			if(color == WHITE) 	Switch3_LED  = 1;
		}
	}		

	// Do stuff here every second
	if( _1second_timer++ >= 38) {
		_1second_timer = 0;
	}


	// If camera was initialized and is ready, then process camera info
	if(Camera_Initialized( )) {

//		Uncomment this code to perform camera exposure test ****************************************************************
//		find_exposure( GREEN );
//		Putdata(&txdata);
//		return;	
		
		// Pan Camera if p1_aux1 is pressed
	  	if(p1_sw_aux1 == 1 ) {							// If p1_aux1 is pressed
			if(p1aux1_debounce++ == 2)					// And pressed for 2 consecutive loops
				 pan_camera( );							// cycle camera left/right/center
		}
		else p1aux1_debounce = 0;						// It's not pressed, reset debounce
	
	
		// Cycle through colors if p1_aux2 is pressed
		if (p1_sw_aux2 == 1) {							// If p1_aux2 is pressed
			if(p1aux2_debounce++ == 2) {				// And pressed for 2 consecutive loops
				if (++color > WHITE) color = RED;		// Don't let selection get > BLUE
				camera_find_color(color);				// Set camera to look for selected color
			}
		}
		else p1aux2_debounce = 0;						// It's not pressed, reset debounce
		  

		// Check if Camera has found the color
	  	if (camera_track_update( )) {					// If we received a packet from camera
			if (cam.count >= CAMERA_MINIMUM_PIXELS)	{ 
				if (tracking_debounce++ >= 3){
					tracking = 1;
					tracking_debounce = 3;

				}			
			}
			else{
				if( tracking_debounce-- <= 0){
					tracking_debounce = 0;
					tracking = 0;
				}
			}
		}


		// Allow camera to control robot if p1 trig is pressed and we are tracking
		if ( p1_sw_trig == 1 ) {
			if( tracking_state == 0) {
				tracking_state = 1;
				color = GREEN;
				vision_sweep_mode = RIGHT;
				goal_sweep_mode = LEFT;
				camera_find_color(color);				// Set camera to look for selected color
				if( vision_sweep_mode == LEFT )
					pan_servo = LEFT_SWEEP_START;
				if( vision_sweep_mode == RIGHT )
					pan_servo = RIGHT_SWEEP_START;
				if( vision_sweep_mode == CENTER )
					pan_servo = CENTER_SWEEP_START;
			}
		}
		else
			tracking_state = 0;







		//********************************************************************************************************
		//
		//
		//							AUTONOMOUS MODE STATE MACHINE
		//
		//		
		//********************************************************************************************************
		// This is the minimum speed applied to the wheels to pivot the robot until the camera pan servo is within the PIVOT_WINDOW.
		// Too large a value will cause robot to pivot faster than the pan servo can react. Too small a value will cause robot never
		// reach the center position. This value is directly related and should be tuned according to the camera's COARSE_TRACKING values for pivoting.
		#define PIVOT_SPEED_RIGHT		85
		#define PIVOT_SPEED_LEFT		170
		
		// PIVOT_SPEED_MIN is increased at this rate by multiplying the pan servo by this value. This causes the robot to pivot faster if the 
		// camera is pointing far away from 127. The further away from 127, the faster the pivoting
		// The bigger the number, the faster we pivot, never above 255, never below PIVOT_SPEED_MIN
		#define PIVOT_MULTIPLIER		1

		// This is the speed at which we start to move towards the target after pivoting
		#define FORWARD_SPEED			180

		// We never slow down below this amount, otherwise we'll waste time
		#define FORWARD_SPEED_MIN       147
	
		#define REVERSE_SPEED_MIN       100

		// After we reach the TILT_SERVO_FINE_SWITCH, slow down at a rate determined by this multiplier.
		// The bigger the number, the faster we slowdown, (sort of the opposite of the PIVOT_MULTIPLIER)
		#define FORWARD_SPEED_MULTIPLIER 4		

		// This corrects the robot steering when the camera's pan servo changes from looking forward. The camera pan servo value is multiplied
		// by this factor and then applied to the wheels. The bigger the multiplier, the move radical the robot corrects, or SWERVES.
		// This value is directly related and should be tuned according to the camera's FINE_TRACKING setting.
		#define	SWERVE_MULTIPLIER		3
		
		// When the camera's  is >= this position (looking down as we approach target), we set the tilt servo to FINE_TRACKING and start
		// to slowdown. The bigger the number, the closer to the target we get before slowing down.
		#define TILT_SERVO_FINE_SWITCH    150

		// When the camera's tilt servo is > this position (looking down as we approach the target), we are at the target.
		// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_BEFORE_POSITION	170

		// When the camera's tilt servo is > this position (looking down as we approach the target), we are at the target.
		// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_AT_POSITION	200

		// This should be the value of the cam.tilt_servo when the arm is lifted with the vision tetra attached. The lower the number the higher up.
		#define TILT_SERVO_TETRA_UP		170
		
		#define TILT_SERVO_FINE_GOAL_SWITCH 145

		#define TILT_SERVO_AT_GOAL      185

		// Defines for states of the Autonomous State Machine
		#define STOP_AND_PAN			1
		#define	CHECK_FOR_TRACKING		2
		#define PIVOT_ROBOT 			3
		#define PIVOT_USING_IMAGE		4
		#define	GO_TO_TARGET			5
		#define	STOP_SHORT				6
		#define	FINAL_PIVOT				7
		#define	STRAIGHT_TO_TARGET		8
		#define AT_TARGET				9
		#define WATCH_IT_RISE			10
		#define WAIT_FOR_BACKUP			11
		#define	ARM_DOWN_DELAY			12
		#define GOT_TETRA_UP			13

		#define FIND_GOAL				14
		#define CHECK_FOR_GOAL_TRACKING 15
		#define	PIVOT_TO_GOAL			16
		#define GO_TO_GOAL				17
		#define	AT_GOAL					18
		#define CAP_IT					19
		#define	AUTONOMOUS_DONE			20


		switch( tracking_state ) {
			//******************************************************************************************************
			// Ignore all activity if state = 0;
			case 0:
			break;

			//******************************************************************************************************
			// First lets make sure we're stopped and cycle camera until we find a target
			case STOP_AND_PAN:
			   	p1_x = 127;												// Stop wheels
		   		p1_y = 127;												// Stop wheels
				// Make sure both camera servos are DISABLED
				if(get_pan_tracking( ) != DISABLE || get_tilt_tracking( ) != DISABLE) {
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(DISABLE, DISABLE);				// Pan.servo = COARSE, Tilt still COARSE
					restart_streaming( );								// Start resending TC packets
				}
				// Now sweep the camer's pan servo position
				printf( "Sweeping for TETRA..." );   
				if( vision_sweep_mode == LEFT ) {						// If we're sweeping LEFT
					printf( "Left  " );   								// print it
					pan_servo -= SWEEP_STEP; 							// step to center
					if(pan_servo < LEFT_SWEEP_MAX)						// if past max position
						pan_servo = LEFT_SWEEP_START;					// reset to left start position
				}
				if( vision_sweep_mode == RIGHT ) {						// If we're sweeping RIGHT		
					printf( "Right  " );   								//  print it
					pan_servo += SWEEP_STEP;							// step to center
					if(pan_servo > RIGHT_SWEEP_MAX)						// if past max position
						pan_servo = RIGHT_SWEEP_START;					// reset to right start position
				} 
				if( vision_sweep_mode == CENTER ) {						// If we're sweeping CENTER
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
			   	p1_x = 127;												// Stop wheels
		   		p1_y = 127;												// Stop wheels
				if( tracking == 0 )										// If camera lost tracking
					tracking_state = STOP_AND_PAN;						// try to aquire again !!!
				else {													// WE are tracking
					printf("Target Found...Entering PIVOT_ROBOT\r");
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(COARSE_TRACKING, DISABLE);		// Pan.servo = COARSE, Tilt,servo = DISABLED
					restart_streaming( );								// Start resending TC packets
					tracking_state = PIVOT_ROBOT;						// Go to next state
				}	
			break;	

			//******************************************************************************************************
			// PIVOT robot until camera is looking straight
			case PIVOT_ROBOT:
				p1_y = 127;												// Don't allow Forward speed
				if( tracking == 0 ) { 									// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");// print it
					tracking_state = STOP_AND_PAN;						// try to aquire again
					break;
				}				
				// Switch to PIVOT_USING_IMAGE when camera is pretty centered
				if( (cam.pan_servo >= 100) && (cam.pan_servo <= 147) ) {// If camera is looking forward
					printf("Now using camera image to center robot...Entering PIVOT_USING_IMAGE\r"); // Print it
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(DISABLE, DISABLE);				// Disable both camera servos.
					camera_set_pan(128); 								// position pan straight ahead, still looking down
					restart_streaming( );								// Start resending TC packets
					tracking_state = PIVOT_USING_IMAGE;					// Starts towards target
					break;
				}
				// Pivot robot towards target
				x = 127;												// start off at zero turning
				
				// Lets PIVOT faster if we're really crooked and slow down as we straighten out by using the multiplier
				// This crazy math keeps p1_x from underflowing below overflowing above 255
				if( cam.pan_servo > 127 ) {							// If we're swerving left
					x -= ((cam.pan_servo - 127) * PIVOT_MULTIPLIER);	// apply multiplier to pivot right
					if( x < 0 ) x = 0;									// Never let x go below zero
					if( x > PIVOT_SPEED_RIGHT)							// If slower than minimum speed
						x = PIVOT_SPEED_RIGHT;							// set speed to minimum speed
					if( old_x != x)										// If we have a change in speed
						printf("Pivoting RIGHT...Speed = %d\r", x);		// Print it out
				}
				// This crazy math keeps p1_x from underflowing below zero
				if( cam.pan_servo < 127 ) {							// If we're swerving right
					x += ((127 - cam.pan_servo) * PIVOT_MULTIPLIER);	// apply multiplier to pivot right
					if( x > 255 ) x = 255;								// never let x go over 255
					if( x < PIVOT_SPEED_LEFT)							// if slower that minimum speed
						x = PIVOT_SPEED_LEFT;							// set speed to minimum
					if( old_x != x )										// if we have a change in speed
						printf("Pivoting LEFT.....Speed = %d\r", x);	// Print it out
				}
				old_x = x;												// save so we can detect a change next loop
				p1_x = x;												// set p1_x to x
				break;

			//******************************************************************************************************
			// PIVOT robot until camera is looking straight
			case PIVOT_USING_IMAGE:
				p1_y = 127;												// Don't allow Forward speed

				if( tracking == 0 ) { 									// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");// print it
					tracking_state = STOP_AND_PAN;						// try to aquire again
					break;
				}				
				// GO_TO_TARGET when really close to target center
				if(cam.x >= 79 && cam.x <= 81) {										// If tetra is centered in camera
					printf("PIVOTING done....Entering GO_TO_TARGET\r"); // Print it
					printf("Goind Forward....Speed = %d\r", FORWARD_SPEED);	// Print it out
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(DISABLE, COARSE_TRACKING);		// Pan.servo = DISABLE, Tilt,servo = COARSE_TRACKING
					restart_streaming( );								// Start resending TC packets
					tracking_state = GO_TO_TARGET;						// Starts towards target
					break;
				}
				// Pivot robot towards target
				if( cam.x > 80 )										// If tetra is left of center
					p1_x = PIVOT_SPEED_RIGHT;							// pivot right
				else													// If tetra is right of center
					p1_x = PIVOT_SPEED_LEFT;							// pivot right
				break;

			//******************************************************************************************************
			// Now that we are looking straight (hopefully)
			case GO_TO_TARGET:
				if( tracking == 0 ) { 										// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// Print it
					tracking_state = STOP_AND_PAN;							// try to aquire again
					break;
				}				

				// Adjust forward speed by using tilt.servo and multiplier if past TILT_SERVO_FINE_SWITCH
				y = FORWARD_SPEED;											// Head forwards towards target								
				// Check if we're pointing down enough to start slowing down
				if (cam.tilt_servo > TILT_SERVO_FINE_SWITCH) {				// If pointing down enough
					if(get_tilt_tracking( ) != FINE_TRACKING ) {			// Make sure TILT servo if FINE_TRACKING
						stop_streaming( );									// Stop sending TC packets ( streaming )
						camera_auto_servo(DISABLE, FINE_TRACKING);			// Keep pan servo disabled, Tilt = FINE Tracking
						restart_streaming( );								// Restart sending TC packets
					}														// END if(get_tilt_tracking( ) != FINE_TRACKING)
					// Slow down robot the closer we get to the target by using tilt servo count and multiplier
					y -= ((cam.tilt_servo - 127) * FORWARD_SPEED_MULTIPLIER); // slow down faster if further away
					if ( y < FORWARD_SPEED_MIN )							// if slower than minimum
						y = FORWARD_SPEED_MIN;								// output minimum speed

					if( old_y != y)											// if we have a change in speed
						printf("Slowing Down.......Speed = %d\r", y);		// Print it out
				}															// END if(cam.tilt_servo > TILT_SERVO_FINE_SWITCH)
				old_y = y;													// save so we can detect a change next loop
				p1_y = y;													// output new forward speed to p1_y


				printf("Camera X = %d\r", cam.x);

				// Lets SWERVE faster if we're really crooked and slow down as we straighten out by using the multiplier
				// This crazy math keeps p1_x from underflowing below overflowing above 255
				x = 127;													// Assume we're going to go straight
				if( cam.x > 80 ) {											// If we're pivoting left
					x -= ((cam.x - 80) * SWERVE_MULTIPLIER);				// apply multiplier to pivot right
					if( x < 0 ) x = 0;										// Never let x go below zero
					if( old_x != x)											// If we have a change in speed
						printf("Swerving RIGHT...Speed = %d\r", x);			// Print it out
				}

				// This crazy math keeps p1_x from underflowing below zero
				if( cam.x < 80 ) {											// If we're pivoting right
					x += ((80 - cam.x) * SWERVE_MULTIPLIER);				// apply multiplier to pivot right
					if( x > 255 ) x = 255;									// never let x go over 255
					if( old_x != x)											// if we have a change in speed
						printf("Swerving LEFT.....Speed = %d\r", x);		// Print it out
				}
				old_x = x;													// save so we can detect a change next loop
				p1_x = x;													// set p1_x to x

				// Check if robot is really close to target..If so then re-check centering
				if(cam.tilt_servo > TILT_SERVO_BEFORE_POSITION) {			// If close enough to target
					printf("Robot @ TILT_SERVO_BEFORE_POSITION...STOPPING to Recheck Centering...Entering STOP_SHORT\r"); // print it
					our_delay = 38;											// Delay before final check
					tracking_state = STOP_SHORT;							// Now stop and wait
				}
				break;

			//******************************************************************************************************
			case STOP_SHORT:
			   	p1_x = 127;													// Stop wheels
		   		p1_y = 127;													// Stop wheels
				if(our_delay-- == 0) {										// if delay is done
					printf("Delay is done...Entering FINAL_PIVOT\r");		// Print it out
					stop_streaming( );										// Stop sending TC packets ( streaming )
					if(get_tilt_tracking( ) != FINE_TRACKING ) {			// Make sure TILT servo if FINE_TRACKING
						stop_streaming( );									// Stop sending TC packets ( streaming )
						camera_auto_servo(DISABLE, FINE_TRACKING);			// Keep pan servo disabled, Tilt = FINE Tracking
						restart_streaming( );								// Restart sending TC packets
					}														// END if(get_tilt_tracking( ) != FINE_TRACKING)
					tracking_state = FINAL_PIVOT;							// check if centered
				}
				break;

			//******************************************************************************************************
			// First state pivots robot until camera is looking straight
			case FINAL_PIVOT:
				p1_y = 127;													// Don't allow Forward speed
				if( tracking == 0 ) { 										// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// print it
					tracking_state = STOP_AND_PAN;							// try to aquire again
					break;
				}				
				if(get_tilt_tracking( ) != FINE_TRACKING ) {				// Make sure TILT servo if FINE_TRACKING
					stop_streaming( );										// Stop sending TC packets ( streaming )
					camera_auto_servo(DISABLE, FINE_TRACKING);				// Keep pan servo disabled, Tilt = FINE Tracking
					restart_streaming( );									// Restart sending TC packets
				}															// END if(get_tilt_tracking( ) != FINE_TRACKING)
				if( cam.x == 80 ) {										// If camera is centered
					printf("Camera Centered..Entering  HIT_TARGET\r");		// Print it out
					tracking_state = STRAIGHT_TO_TARGET;					// Starts towards target
				}
				// Pivot robot towards target
				if( cam.x > 80 )											// If tetra is left of center
					p1_x = PIVOT_SPEED_RIGHT;								// pivot right
				else														// If tetra is right of center
					p1_x = PIVOT_SPEED_LEFT;								// pivot right
				break;

			//******************************************************************************************************
			case STRAIGHT_TO_TARGET:
				if( tracking == 0 ) { 										// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");	// Print it
					tracking_state = STOP_AND_PAN;							// try to aquire again
					break;
				}				
				p1_x = 127;													// No left or right motion
				p1_y = FORWARD_SPEED_MIN;									// Head forwards towards target								
				if(cam.tilt_servo > TILT_SERVO_AT_POSITION) {				// if close enough to target
					printf("Robot @ TILT_SERVO_AT_POSITION...Entering AT_TARGET\r"); // print it
					tracking_state = AT_TARGET;								// Go to next state
				}
				break;

			//******************************************************************************************************
			// Now we are at the target
			case AT_TARGET:
			   	p1_x = 127;													// Stop wheels
	 	  		p1_y = 127;													// Stop wheels
				stop_streaming( );											// Stop sending TC packets ( streaming )
				camera_auto_servo(DISABLE, SUPER_COARSE_TRACKING);			// Pan.servo = DISABLE, tilt.servo = SUPER_COARSE_TRACKING
				restart_streaming( );										// Start resending TC packets
				printf("Lifting ARM....Entering WATCH_IT_RISE\r");			// print it	
				arm_up( );													// send ARM up
				got_tetra = 0;												// clear got_tetra flag
				tracking_state = WATCH_IT_RISE;
				break;

			//******************************************************************************************************
			case WATCH_IT_RISE:
				arm_up( );													// send ARM up
				if(cam.tilt_servo < TILT_SERVO_TETRA_UP )					// if the tilt servo is higher than the TILT_SERVO_TETRA up position
					got_tetra = 1;											// set the tetra_up flag
				if ( HANG_TOP_LIMIT == 0 ) { 								// if ARM up sensor blocked
					if( got_tetra == 1) {									// if we saw the tetra rise
						printf("Got Vision Tetra\r");						// print it
						tracking_state = GOT_TETRA_UP;						// Success...now find the goal
					}														
					// SHIT!!!  We missed the tetra... Start the recovery code
					else {													// Did not see the tetra
						printf("Missed it...Backing up to TILT_SERVO_BEFORE_POSITION\r"); // print it
						stop_streaming( );									// Stop sending TC packets ( streaming )
						camera_auto_servo(DISABLE, FINE_TRACKING);			// Pan.servo = DISABLE, tilt.servo = FINE_TRACKING
						restart_streaming( );								// Start resending TC packets
						tracking_state = WAIT_FOR_BACKUP;					// startup the recovery code
					}
				}															// END if (HANG_TOP_LIMIT == 0)
				break;

			//******************************************************************************************************
			//   WE MISSED THE TETRA.....BEGIN BACKUP and RECOVERY CODE
			//******************************************************************************************************
			case WAIT_FOR_BACKUP:
				p1_x = 127;													// stop left and right motion
				p1_y = REVERSE_SPEED_MIN;									// stop forward motion
				if(cam.tilt_servo < TILT_SERVO_BEFORE_POSITION) {			// if we backed up far enough
					HANG_MOTOR = HANG_DOWN_SPEED;							// Lower ARM
					printf("Lowering ARM\r");								// print it
					our_delay = 38;											// ARM down time
					tracking_state = ARM_DOWN_DELAY;						// go to next state
				}
				break;

			//******************************************************************************************************
			case ARM_DOWN_DELAY:
			   	p1_x = 127;													// stop all motion
		   		p1_y = 127;													// stop all motion
				HANG_MOTOR = HANG_DOWN_SPEED;								// Arm down speed
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
				p1_x = 127;													// Stop wheels
		   		p1_y = 127;													// Stop wheels
				arm_up( );
				if( goal_sweep_mode == LEFT )
					pan_servo = LEFT_SWEEP_START;			
				if( goal_sweep_mode == RIGHT )
					pan_servo = RIGHT_SWEEP_START;			
				if( goal_sweep_mode == CENTER )
					pan_servo = CENTER_SWEEP_START;			
				camera_find_color (GREEN);
				tracking_state = FIND_GOAL;
				break;

			//******************************************************************************************************
			case FIND_GOAL:
				p1_x = 127;													// Stop wheels
		   		p1_y = 127;													// Stop wheels
				arm_up( );
				if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != DISABLE) {
					stop_streaming( );										// Stop sending TC packets ( streaming )
					camera_auto_servo(COARSE_TRACKING, DISABLE);			// Pan.servo = COARSE, Tilt Disable
					restart_streaming( );									// Start resending TC packets
				}
				printf( "Sweeping for GOAL..." );   
				if( goal_sweep_mode == LEFT ) {
					printf( "Left  " );   
					pan_servo -= SWEEP_STEP; 
					if(pan_servo < LEFT_SWEEP_MAX)
						pan_servo = LEFT_SWEEP_START;
				}
				if( goal_sweep_mode == RIGHT ) {
					printf( "Right  " );   
					pan_servo += SWEEP_STEP;
					if(pan_servo > RIGHT_SWEEP_MAX)
						pan_servo = RIGHT_SWEEP_START;
				} 
				if( goal_sweep_mode == CENTER ) {
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
	
			case CHECK_FOR_GOAL_TRACKING:
			   	p1_x = 127;												// Stop wheels
		   		p1_y = 127;												// Stop wheels
				arm_up( );
				if( tracking == 0 )										// If camera lost tracking
					tracking_state = FIND_GOAL;							// try to aquire again !!!
				else {													// WE are tracking
					printf("GOAL Found...PIVOTING robot...Entering PIVOT_TO_GOAL\r");
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(COARSE_TRACKING, DISABLE);		// Pan.servo = COARSE, Tilt,servo = DISABLED
					restart_streaming( );								// Start resending TC packets
					tracking_state = PIVOT_TO_GOAL;						// Go to next state
				}	
			break;	


			case PIVOT_TO_GOAL:
				if( tracking == 0 ) { 									// If we lost tracking
					printf("Lost Tracking....Entering  FIND_GOAL\r");
					tracking_state = FIND_GOAL;						    // try to aquire again
					break;
				}				

				arm_up( );
				// GO_TO_GOAL when really close to target center
				if( (cam.pan_servo > 125) && (cam.pan_servo < 129) ) {	// If camera is looking forward
					if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
						stop_streaming( );									// Stop sending TC packets ( streaming )
						camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);// Pan.servo = COARSE, Tilt still COARSE
						restart_streaming( );								// Start resending TC packets
					}
					printf("Pivot done...Going to target\r");				// Print it
					tracking_state = GO_TO_GOAL;							// Starts towards target
					break;
				}

				// Pivot robot towards target
				p1_y = 127;											// Don't allow Forward speed
				x = 127;											// start off at zero turning
				
				// Lets PIVOT faster if we're really crooked and slow down as we straighten out
				// This crazy math keeps p1_x from underflowing below overflowing above 255
				if( cam.pan_servo > 127 ) {							// If we're swerving left
					x -= ((cam.pan_servo - 127) * PIVOT_MULTIPLIER);// apply multiplier to pivot right
					if( x < 0 ) x = 0;								// Never let x go below zero
					if( x > PIVOT_SPEED_RIGHT)						// If slower than minimum speed
						x = PIVOT_SPEED_RIGHT;						// set speed to minimum speed
					if( old_x != x)									// If we have a change in speed
						printf("Pivoting RIGHT %d\r", x);			// Print it out
				}
				// This crazy math keeps p1_x from underflowing below zero
				if( cam.pan_servo < 127 ) {							// If we're swerving right
					x += ((127 - cam.pan_servo) * PIVOT_MULTIPLIER);// apply multiplier to pivot right
					if( x > 255 ) x = 255;							// never let x go over 255
					if( x < PIVOT_SPEED_LEFT)						// if slower that minimum speed
						x = PIVOT_SPEED_LEFT;						// set speed to minimum
					if( old_x != x)									// if we have a change in speed
						printf("Pivoting LEFT %d\r", x);			// Print it out
				}
				old_x = x;											// save so we can detect a change next loop
				p1_x = x;											// set p1_x to x
				break;


	
			case GO_TO_GOAL:
				if( tracking == 0 ) { 											// If we lost tracking
					printf("Lost Tracking....Entering  FIND_GOAL\r");			// Print it
					tracking_state = FIND_GOAL; 								// try to aquire again
					break;
				}			
				arm_up( );
				// Ok camera is still within MAX swerve, now adjust course if necessary
				y = FORWARD_SPEED;												// Head forwards towards target								
				x = 127;														// Assume we're going to go straight

				// Check if we're pointing down enough to start slowing down
				if (cam.tilt_servo > TILT_SERVO_FINE_GOAL_SWITCH) {				// If pointing down enough
					if(get_tilt_tracking( ) != FINE_TRACKING ) {				// Make sure PAN servo if FINE_TRACKING
						stop_streaming( );										// Stop sending TC packets ( streaming )
						camera_auto_servo(FINE_TRACKING, FINE_TRACKING);		// Pan and Tilt = FINE Tracking
						restart_streaming( );									// Restart sending TC packets
					}
					// Slow down robot the closer we get to the target by using tilt servo count and multiplier
					y -= ((cam.tilt_servo - 127) * FORWARD_SPEED_MULTIPLIER);
					if ( y < FORWARD_SPEED_MIN )								// if slower than minimum
						y = FORWARD_SPEED_MIN;									// output minimum speed
				}
				
				// This crazy math keeps p1_x from overflowing above 255
				if( cam.pan_servo > 127 ) {										// If we're swerving right
					x -= ((cam.pan_servo - 127) * SWERVE_MULTIPLIER);			// apply multiplier to swerve left
					if( x < 0 ) x = 0;											// Never let x go below zero
					if( old_x != x)												// If we have a change in speed
						printf("Swerving RIGHT %d\r", x);						// Print it out
				}
				// This crazy math keeps p1_x from underflowing below zero
				if( cam.pan_servo < 127 ) {										// If we're swerving left
					x += ((127 - cam.pan_servo) * SWERVE_MULTIPLIER);			// swerve left
					if( x > 255 ) x = 255;										// never let x go over 255
					if( old_x != x)												// if we have a change in speed
						printf("Swerving LEFT %d\r", x);						// Print it out
				}
				old_x = x;														// save so we can detect a change next loop
				p1_x = x;														// set p1_x to x
				p1_y = y;														// set p1_y to y

				if(cam.tilt_servo > TILT_SERVO_AT_GOAL) {					// if close enough to target
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
				p1_x = 127;														// stop all motion
			   	p1_y = 127;														// stop all motion
				printf("Lowering ARM....Entering CAP_IT\r");	
				our_delay = 38;							// arm down time
				tracking_state = CAP_IT;		// go to next state
			break;


			//******************************************************************************************************
			case CAP_IT:
			   	p1_x = 127;									// stop all motion
		   		p1_y = 127;									// stop all motion
				HANG_MOTOR = HANG_DOWN_SPEED;				// Arm down speed
		      	if (our_delay-- == 0) {						// If delay is done			
					printf("Arm is down ARM...We are capped!!....Entering AUTONOMOUS_DONE\r");
					tracking_state = AUTONOMOUS_DONE;
				}
			break;


			//******************************************************************************************************
			case AUTONOMOUS_DONE:
				p1_x = 127;
				p1_y = 127;
				HANG_MOTOR = 127;							// Arm down speed
			break;
	
		}  // END switch( tracking_state )				

		//********************************************************************************************************
		//
		//
 		//							END OF AUTONOMOUS MODE STATE MACHINE
		//
		//		
		//********************************************************************************************************

	}

	if(dual_joysticks == 1)								// If using dual joysticks	
	{
		LEFT_WHEEL =  p1_y;								// Simply output p1_y to LEFT wheel
		RIGHT_WHEEL = p2_y;								// Simply output p2_y to RIGHT wheel
	}
	else												// else combine X-Y to single joystick drive
	{
	   	LEFT_WHEEL  = Limit_Mix(2000 + p1_y + p1_x - 127);
		RIGHT_WHEEL = Limit_Mix(2000 + p1_y - p1_x + 127);
	}

	if( Accelleration != 0 )							// If accelleration limiting is required
		SoftStartWheels( );								// Prevent rapid changes @ wheels to reduce slippage

	
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



/*******************************************************************************
* FUNCTION NAME: pan camera
* PURPOSE:       Cycles camera LEFT / RIGHT / CENTER
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void pan_camera( void )
{

	if (++pan_dir > 2) pan_dir = 0;
	
	switch(pan_dir) {
		case 0: 
		printf( "Moving Camera Left\r");   
		camera_set_servos( 77, 128 ); 
		break;

		case 1: 
		printf( "Moving Camera Center\r");   
		camera_set_servos( 128, 128 ); 
		break;
	
		case 2: 
		printf( "Moving Camera Right\r");   
		camera_set_servos( 177, 128 ); 
		break;
	}
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



int read_selector_switch( void )
{
	int x = 0;
	
	if( SELECTOR_SW_BIT8 == 0 )
		x += 8;
	if( SELECTOR_SW_BIT4 == 0 )
		x += 4;
	if( SELECTOR_SW_BIT2 == 0 )
		x += 2;
	if( SELECTOR_SW_BIT1 == 0 )
		x += 1;
	return x;
}



void arm_up( void )
{
	HANG_MOTOR = HANG_UP_SPEED;
		if ( HANG_TOP_LIMIT == 0 ) 	{			
			HANG_MOTOR = HANG_UP_HOLD_SPEED;
		}
}			
