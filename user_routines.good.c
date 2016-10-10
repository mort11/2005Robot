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
int 			our_delay, x, old_x, y, old_y, old_pan_servo;
int				pan_servo_copy, tilt_servo_copy;
int 			pan_servo;
int 			sweep_mode;
int 			tracking_debounce = 0;

#define LEFT 	0
#define RIGHT	1
#define CENTER	2


#define LEFT_SWEEP_START  	180
#define LEFT_SWEEP_MAX  	100
#define RIGHT_SWEEP_START  	80
#define RIGHT_SWEEP_MAX  	160
#define SWEEP_STEP			5
#define CENTER_SWEEP_START  127



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
				if(HANG_JOYSTICK > 145)
					HANG_JOYSTICK = 145;
			
				if(HANG_JOYSTICK < 100)
					HANG_JOYSTICK = 100;
			
				HANG_MOTOR = HANG_JOYSTICK;
				
				if ( HANG_BOTTOM_LIMIT == 0 && HANG_JOYSTICK > 127 )
					HANG_MOTOR = 127;
			
				if ( HANG_TOP_LIMIT == 0 && HANG_JOYSTICK < 127 )
					HANG_MOTOR = 127;
			

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

//		Uncomment this code to perform camera exposure test
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
					pan_servo_copy  = cam.pan_servo;
					tilt_servo_copy = cam.tilt_servo; 
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
				sweep_mode = RIGHT;
				camera_find_color(color);				// Set camera to look for selected color
				if( sweep_mode == LEFT )
					pan_servo = LEFT_SWEEP_START;
				if( sweep_mode == RIGHT )
					pan_servo = RIGHT_SWEEP_START;
				if( sweep_mode == CENTER )
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
		#define PIVOT_SPEED_RIGHT	85
		#define PIVOT_SPEED_LEFT	170
		
		// PIVOT_SPEED_MIN is increased at this rate by multiplying the pan servo by this value. This causes the robot to pivot faster if the 
		// camera is pointing far away from 127. The further away from 127, the faster the pivoting
		// The bigger the number, the faster we pivot, never above 255, never below PIVOT_SPEED_MIN
		#define PIVOT_MULTIPLIER		1

		// This is the speed at which we start to move towards the target after pivoting
		#define FORWARD_SPEED		 	80

		// We never slow down below this amount, otherwise we'll waste time
		#define FORWARD_SPEED_MIN       45
	
		#define REVERSE_SPEED_MIN       100

		// After we reach the TILT_SERVO_FINE_SWITCH, slow down at a rate determined by this multiplier.
		// The bigger the number, the faster we slowdown, (sort of the opposite of the PIVOT_MULTIPLIER)
		#define FORWARD_SPEED_MULTIPLIER 4		

		// This corrects the robot steering when the camera's pan servo changes from looking forward. The camera pan servo value is multiplied
		// by this factor and then applied to the wheels. The bigger the multiplier, the move radical the robot corrects, or SWERVES.
		// This value is directly related and should be tuned according to the camera's FINE_TRACKING setting.
		#define	SWERVE_MULTIPLIER		5
		
		// This is the maximum camera's pan max servo value variation from 127 before stopping and re-aquiring the target.
		// Keeps robot from failing to track the target and smashing into something. 
		#define	MAX_SWERVE				100

		// When the camera's  is >= this position (looking down as we approach target), we set the tilt servo to FINE_TRACKING and start
		// to slowdown. The bigger the number, the closer to the target we get before slowing down.
		#define TILT_SERVO_FINE_SWITCH  155

		// When the camera's tilt servo is > this position (looking down as we approach the target), we are at the target.
		// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_BEFORE_POSITION	170

		// When the camera's tilt servo is > this position (looking down as we approach the target), we are at the target.
		// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_AT_POSITION	195

		// This should be the value of the cam.tilt_servo when the arm is lifted with the vision tetra attached. The lower the number the higher up.
		#define TILT_SERVO_TETRA_UP		127

		// Defines for states of the Autonomous State Machine
		#define STOP_AND_PAN		1
		#define WAIT_FOR_DELAY		2
		#define	CHECK_FOR_TRACKING	3
		#define PIVOT_ROBOT 		4
		#define	GO_TO_TARGET		5
		#define	DELAY_FOR_PIVOT		6
		#define	RE_PIVOT_ROBOT		7
		#define	STRAIGHT_TO_TARGET	8
		#define AT_TARGET			9
		#define LIFT_ARM			10
		#define WATCH_IT_RISE		11
		#define WAIT_FOR_BACKUP		12
		#define	ARM_DOWN_DELAY		13
		#define GOT_TETRA_UP		14


// IRELAND/SANCHEZ/BROCKHOFF/SACCO
// NEEDS TO BE CODED		
		#define FIND_GOAL			15
		#define	PIVOT_TO_GOAL		16
		#define GO_TO_GOAL			17
		#define	AT_GOAL				18
		#define CAP_IT				19
		#define	AUTONOMOUS_DONE		20

		
		switch( tracking_state ) {
			//******************************************************************************************************
			// Ignore all activity if state = 0;
			case 0:
			break;

			//******************************************************************************************************
			// First lets make sure we're stopped and cycle camera until we find a target
			case STOP_AND_PAN:
			   	p1_x = 127;								// Stop wheels
		   		p1_y = 127;								// Stop wheels
				// Make sure both servos are COARSE_TRACKING
				if(get_pan_tracking( ) != DISABLE && get_tilt_tracking( ) != DISABLE) {
					printf("Disabling AUTO servo\r");	// Print it out
					stop_streaming( );									// Stop sending TC packets ( streaming )
					camera_auto_servo(DISABLE, DISABLE);// Pan.servo = COARSE, Tilt still COARSE
					restart_streaming( );								// Start resending TC packets
				}

				printf( "Sweeping..." );   
				if( sweep_mode == LEFT ) {
					printf( "Left  " );   
					pan_servo -= SWEEP_STEP; 
					if(pan_servo < LEFT_SWEEP_MAX)
						pan_servo = LEFT_SWEEP_START;
				}
				if( sweep_mode == RIGHT ) {
					printf( "Right  " );   
					pan_servo += SWEEP_STEP;
					if(pan_servo > RIGHT_SWEEP_MAX)
						pan_servo = RIGHT_SWEEP_START;
				} 
				printf("%d\r", pan_servo);

				stop_streaming( );						// Stop sending TC packets ( streaming )
				camera_set_servos( pan_servo, 140 ); 
				restart_streaming( );					// Start resending TC packets
				our_delay = 0;							// set delay to count 18 26.2 ms loops ( or 1/2 second )
				tracking_state = WAIT_FOR_DELAY;		// go to next state
			break;
	
			//******************************************************************************************************
			case WAIT_FOR_DELAY:
			   	p1_x = 127;								// Stop wheels
		   		p1_y = 127;								// Stop wheels
		      	if (our_delay-- <= 0)					// If delay is done			
					tracking_state = CHECK_FOR_TRACKING;// go to next state
			break;	

			//******************************************************************************************************
			case CHECK_FOR_TRACKING:
			   	p1_x = 127;									// Stop wheels
		   		p1_y = 127;									// Stop wheels
				if( tracking == 0 )							// If camera lost tracking
					tracking_state = STOP_AND_PAN;			// try to aquire again
				else {										// WE are tracking
					printf("Target Found, Enabling AUTO servo and PIVOTING robot\r");
					stop_streaming( );						// Stop sending TC packets ( streaming )
					camera_auto_servo(COARSE_TRACKING, DISABLE);// Pan.servo = COARSE, Tilt,servo = DISABLED
					restart_streaming( );					// Start resending TC packets
					tracking_state = PIVOT_ROBOT;			// Go to next state
				}	
			break;	

			//******************************************************************************************************
			// First state pivots robot until camera is looking straight
			case PIVOT_ROBOT:
				if( tracking == 0 ) { 										// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");
					tracking_state = STOP_AND_PAN;							// try to aquire again
					break;
				}				

				// GO_TO_TARGET when really close to target center
				if( (pan_servo_copy > 125) && (pan_servo_copy < 129) ) {	// If camera is looking forward
					if(get_pan_tracking( ) != COARSE_TRACKING || get_tilt_tracking( ) != COARSE_TRACKING) {
						stop_streaming( );									// Stop sending TC packets ( streaming )
						camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);// Pan.servo = COARSE, Tilt still COARSE
						restart_streaming( );								// Start resending TC packets
					}
					printf("Pivot done...Going to target\r");				// Print it
					tracking_state = GO_TO_TARGET;							// Starts towards target
					break;
				}
				// Set pan.servo to FINE_TRACKING if close enough to target center
//				if( (pan_servo_copy > 113) && (pan_servo_copy < 141) ) {	// If camera is looking forward
//					if(get_pan_tracking( ) != FINE_TRACKING) {
//						stop_streaming( );									// Stop sending TC packets ( streaming )
//						camera_auto_servo(FINE_TRACKING, COARSE_TRACKING);	// Pan.servo = FINE, Tilt still COARSE
//						restart_streaming( );								// Start resending TC packets
//					}
//				}

				// Pivot robot towards target
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
				if( tracking == 0 ) { 									// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");// Print it
					tracking_state = STOP_AND_PAN;						// try to aquire again
					break;
				}				
				// Check if we are out of control by comparing cam pan servo with max swerve
				if( pan_servo_copy > 200 || pan_servo_copy < 25 ) {
					printf("*** ABORT**** At MAX_SWERVE....Entering  STOP_AND_PAN\r");
					tracking_state = STOP_AND_PAN;								// Try to aquire again
					break;
				}
				// Ok camera is still within MAX swerve, now adjust course if necessary
				y = 127 + FORWARD_SPEED;										// Head forwards towards target								
				x = 127;														// Assume we're going to go straight

				// Check if we're pointing down enough to start slowing down
				if (tilt_servo_copy > TILT_SERVO_FINE_SWITCH) {					// If pointing down enough
					if(get_tilt_tracking( ) != FINE_TRACKING ) {				// Make sure PAN servo if FINE_TRACKING
						printf("At TILT_SERVO FINE SWITCH....Setting FINE_TRACKING for Tilt Servo and slowing down\r");
						stop_streaming( );										// Stop sending TC packets ( streaming )
						camera_auto_servo(COARSE_TRACKING, FINE_TRACKING); 		// Pan and Tilt = FINE Tracking
						restart_streaming( );									// Restart sending TC packets
					}
					// Slow down robot the closer we get to the target by using tilt servo count and multiplier
					y -= ((tilt_servo_copy - 127) * FORWARD_SPEED_MULTIPLIER);
					if (y < (127 + FORWARD_SPEED_MIN))
						y = 127 + FORWARD_SPEED_MIN;
				}
				
				// This crazy math keeps p1_x from overflowing above 255
				if( pan_servo_copy > 127 ) {									// If we're swerving right
					x -= ((pan_servo_copy - 127) * SWERVE_MULTIPLIER);			// apply multiplier to swerve left
					if( x < 0 ) x = 0;											// Never let x go below zero
					if( old_x != x)												// If we have a change in speed
						printf("Swerving RIGHT %d\r", x);						// Print it out
				}
				// This crazy math keeps p1_x from underflowing below zero
				if( pan_servo_copy < 127 ) {									// If we're swerving left
					x += ((127 - pan_servo_copy) * SWERVE_MULTIPLIER);			// swerve left
					if( x > 255 ) x = 255;										// never let x go over 255
					if( old_x != x)												// if we have a change in speed
						printf("Swerving LEFT %d\r", x);						// Print it out
				}
				old_x = x;														// save so we can detect a change next loop
				p1_x = x;														// set p1_x to x
				p1_y = y;														// set p1_y to y

				if(tilt_servo_copy > TILT_SERVO_BEFORE_POSITION) {				// If close enough to target
//					if( pan_servo_copy > 125 && pan_servo_copy < 129 ) {		// If looking straight ahead
//						printf("Camera Centered..Entering  STRAIGHT_TO_TARGET\r");		// Print it out
//						tracking_state = STRAIGHT_TO_TARGET;					// Starts towards target
//						break;
//					}
					printf("Robot not in position.. STOPPING and REPIVOTING...\r"); // print it
					our_delay = 19;
					tracking_state = DELAY_FOR_PIVOT;
				}
				break;

			//******************************************************************************************************
			case DELAY_FOR_PIVOT:
			p1_y = 127;
			p1_x = 127;
			if(our_delay-- == 0) {
				printf("Delay is done...Setting FINE_TRACKING and RE-PIVOTING\r");	// Print it out
				stop_streaming( );													// Stop sending TC packets ( streaming )
				camera_auto_servo(FINE_TRACKING, FINE_TRACKING);					// Pan.servo = COARSE, Tilt still COARSE
				restart_streaming( );												// Start resending TC packets
				tracking_state = RE_PIVOT_ROBOT;
			}
			break;

			//******************************************************************************************************
			// First state pivots robot until camera is looking straight
			case RE_PIVOT_ROBOT:
				if( tracking == 0 ) { 										// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");
					tracking_state = STOP_AND_PAN;							// try to aquire again
					break;
				}				
				if( pan_servo_copy > 125 && pan_servo_copy < 129 ) {
					printf("Camera Centered..Entering  HIT_TARGET\r");		// Print it out
					tracking_state = STRAIGHT_TO_TARGET;					// Starts towards target
				}

				// Pivot robot towards target
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
			case STRAIGHT_TO_TARGET:
				if( tracking == 0 ) { 											// If we lost tracking
					printf("Lost Tracking....Entering  STOP_AND_PAN\r");		// Print it
					tracking_state = STOP_AND_PAN;								// try to aquire again
					break;
				}				
				p1_y = 127 + FORWARD_SPEED_MIN;									// Head forwards towards target								
				p1_x = 127;														// Assume we're going to go straight

				if(tilt_servo_copy > TILT_SERVO_AT_POSITION) {					// if close enough to target
					printf("AT TARGET....ALL STOP...\r");						// print it
					tracking_state = AT_TARGET;									// what is next ?
				}
				break;

			//******************************************************************************************************
			// Now we are at the target
			case AT_TARGET:
			   	p1_x = 127;														// stop all motion
		   		p1_y = 127;														// stop all motion
				tracking_state = LIFT_ARM;
				printf("Lifting ARM\r");	
				break;

			//******************************************************************************************************
			case LIFT_ARM:
			   	p1_x = 127;														// stop all motion
		   		p1_y = 127;														// stop all motion
				printf("Setting both servos to COARSE_TRACKING\r");				// Print it out
				printf("Lifting ARM\r");	
				if(get_tilt_tracking( ) != COARSE_TRACKING ) {
					stop_streaming( );												// Stop sending TC packets ( streaming )
					camera_auto_servo(COARSE_TRACKING, COARSE_TRACKING);			// Pan.servo = COARSE, Tilt still COARSE
					restart_streaming( );											// Start resending TC packets
				}
				tracking_state = WATCH_IT_RISE;
				camera_auto_servo(COARSE_TRACKING, SUPER_COARSE_TRACKING);			// Pan.servo = COARSE, Tilt still COARSE
				break;

			//******************************************************************************************************
			case WATCH_IT_RISE:
				HANG_MOTOR = 90; 
				if (HANG_TOP_LIMIT == 0) {
					HANG_MOTOR = 127;
					if(tilt_servo_copy < TILT_SERVO_TETRA_UP) {					// If we have the tetra	
						printf("Got Vision Tetra\r");	
						tracking_state = GOT_TETRA_UP;
						break;			
					}
					our_delay = 38;							// backup time
					printf("Missed it...Backing up\r");	
					tracking_state = WAIT_FOR_BACKUP;	
				}
				break;

			//******************************************************************************************************
			case WAIT_FOR_BACKUP:
				p1_x = 127;
				p1_y = REVERSE_SPEED_MIN;					// backup speed
		      	if (our_delay-- == 0) {						// If delay is done			
					printf("Lowering ARM\r");	
					our_delay = 38;							// arm down time
					tracking_state = ARM_DOWN_DELAY;		// go to next state
				}
				break;

			//******************************************************************************************************
			case ARM_DOWN_DELAY:
			   	p1_x = 127;									// stop all motion
		   		p1_y = 127;									// stop all motion
				HANG_MOTOR = 127;							// Arm down speed
		      	if (our_delay-- == 0) {						// If delay is done			
					printf("Arm is down ARM...Trying again\r");	
					tracking_state = CHECK_FOR_TRACKING;
				}
				break;

			//******************************************************************************************************
			case GOT_TETRA_UP:
				break;

// IRELAND/SANCHEZ/BROCKHOFF/SACCO
// NEEDS TO BE CODED
			case FIND_GOAL:
			break;
	
			case PIVOT_TO_GOAL:
			break;
	
			case GO_TO_GOAL:
			break;
	
			case AT_GOAL:
			break;
	
			case CAP_IT:
			break;
	
			case AUTONOMOUS_DONE:
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




