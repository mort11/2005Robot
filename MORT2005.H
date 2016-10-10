/*******************************************************************************
* FILE NAME: mort2005.h
*
* DESCRIPTION: 
*  This is the include file that defines all I/O used in the MORT 2005 Robot
*  It contains aliases and function prototypes used in all MORT 'C' routines.
*
*******************************************************************************/

#ifndef __mort2005_h_
#define __mort2005_h_

/*************************************************************************************
			                        MORT DEFINES
*************************************************************************************/

/**********************************CAMERA DEFINES************************************/
// These are the exposure variables for each competition
#define FIRST_RED	 20
#define FIRST_GREEN  13
#define FIRST_BLUE 	 50
#define FIRST_YELLOW 08
#define FIRST_WHITE  50

/************************************PWM DEFINES*************************************/
// Drive PWM allocations
#define	LEFT_WHEEL						pwm01			// PWM for the left drive motor
#define	RIGHT_WHEEL						pwm03			// PWM for the right drive motor

// End effector PWM allocations
#define TOWER_MOTOR  					pwm05			// PWM for the tower motor

/***********************************INPUT DEFINES************************************/
// End effector Joystick allocations
#define TOWER_JOYSTICK				  	p3_y			// Joystick control for tower

/*******************************DIGITAL INPUT DEFINES********************************/
// Endefector Digital Input Assignments
#define TOWER_CHANNEL_A				rc_dig_in02  		// TOWER shaft encoder channel A
#define TOWER_CHANNEL_B				rc_dig_in04  		// TOWER shaft encoder channel B
#define TOWER_HOME_SWITCH    		rc_dig_in05			// TOWER Bottom Switch
#define TOWER_TOP_LIMIT				rc_dig_in06			// TOWER Top Limit

// Miscellanous Assignments

/*******************************ANALOG DEFINES*************************************/
// Autonomous Mode Assignments
#define MODE_SWITCH_A2  			rc_dig_in10     	// Selects bit 2 of autonomous mode 
#define MODE_SWITCH_A1	   			rc_dig_in11     	// Selects bit 1 of autonomous mode
#define MODE_SWITCH_A0				rc_dig_in12     	// Selects bit 0 of autonomous mode

// Autonomous Vision Tetra Sweep Assignments
#define VISION_SWEEP_A1  			rc_dig_in13     	// Selects bit 1 of vision tetra sweep
#define VISION_SWEEP_A0				rc_dig_in14     	// Selects bit 0 of vision tetra sweep

// Autonomous Vision Goal Sweep Assignments
#define GOAL_SWEEP_A1				rc_dig_in15     	// Selects bit 1 of goal sweep
#define GOAL_SWEEP_A0				rc_dig_in16			// Selects bit 0 of goal sweep

/*******************************CONSTANT DEFINES*************************************/
// Camera Constants 
#define	CAMERA_MINIMUM_PIXELS	1						// This determines the minimum of tracked pixels to avoid 
														// "chasing noise" or hats in the upper row
// End effector Constants
#define TOWER_UP_SPEED			127						// Speed the tower goes up
#define	TOWER_DOWN_SPEED		20						// Speed the tower goes down 
#define IPP						.0625					// Inches per pulse of rotation counter

// Miscellaneous Constants
#define ON 		1
#define OFF 	0

/**********************************RELAY DEFINES*************************************/
// Miscellaneous Aliases 

/*****************************FUNCTION PROTOTYPES*************************************/
// Interrupt Prototypes 
void Int_1_Handler(void);
void Int_2_Handler(void);
void Initialize_Interrupts(void);  

#endif
