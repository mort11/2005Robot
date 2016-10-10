#include "MORT2005.h"


// Defines for panning the camera to look for Tetras or Goals
		#define SWEEP_STEP			5

		#define LEFT_SWEEP_START  	255
		#define LEFT_SWEEP_MAX  	100

		#define RIGHT_SWEEP_START  	60
		#define RIGHT_SWEEP_MAX  	180

		#define CENTER_SWEEP_START  100
		#define CENTER_SWEEP_MAX  	150



// Defines for states of the Autonomous State Machine looking for vision tetra
		#define GO_LOAD_HEIGHT			1
		#define GO_LOAD_DELAY			2
		#define STOP_AND_PAN			3
		#define	CHECK_FOR_TRACKING		4
		#define PIVOT_ROBOT 			5
		#define	GO_TO_TARGET			6
		#define	STOP_SHORT				7
		#define	FINAL_PIVOT				8
		#define	STRAIGHT_TO_TARGET		9
		#define AT_TETRA				10
		#define WATCH_IT_RISE			11
		#define WAIT_FOR_BACKUP			12
		#define	ARM_DOWN_DELAY			13
		#define GOT_TETRA_UP			14
		#define FIND_GOAL				15
		#define CHECK_FOR_GOAL_TRACKING 16
		#define	PIVOT_TO_GOAL			17
		#define GO_TO_GOAL				18
		#define	AT_GOAL					19
		#define	FINAL_GOAL_PIVOT		20
		#define	GO_GOAL_HEIGHT			21
		#define	GO_GOAL_DELAY			22
		#define	GO_A_LITTLE_FORWARD		23
		#define	A_LITTLE_DELAY			24
		#define CAP_IT					25
		#define CAP_IT_DELAY			26
		#define	GO_A_LITTLE_REVERSE		27
		#define	REVERSE_DELAY			28
		#define	AUTONOMOUS_DONE			29


// This is the minimum speed applied to the wheels to pivot the robot until the camera pan servo is within the PIVOT_WINDOW.
// Too large a value will cause robot to pivot faster than the pan servo can react. Too small a value will cause robot never
// reach the center position. This value is directly related and should be tuned according to the camera's COARSE_TRACKING values for pivoting.
		#define PIVOT_SPEED_RIGHT		105
		#define PIVOT_SPEED_LEFT		150


// This is the speed at which we start to move towards the target after pivoting
		#define FORWARD_SPEED		 	60

// We never slow down below this amount, otherwise we'll waste time
		#define FORWARD_SPEED_MIN       30
		#define REVERSE_SPEED_MIN       30

// After we reach the TILT_SERVO_FINE_SWITCH, slow down at a rate determined by this multiplier.
// The bigger the number, the faster we slowdown, (sort of the opposite of the PIVOT_MULTIPLIER)
		#define FORWARD_SPEED_MULTIPLIER 4		

// This corrects the robot steering when the camera's pan servo changes from looking forward. The camera pan servo value is multiplied
// by this factor and then applied to the wheels. The bigger the multiplier, the move radical the robot corrects, or SWERVES.
// This value is directly related and should be tuned according to the camera's FINE_TRACKING setting.
		#define	SWERVE_MULTIPLIER		1

// This is the maximum camera's pan max servo value variation from 127 before stopping and re-aquiring the target.
// Keeps robot from failing to track the target and smashing into something. 
		#define	MAX_SWERVE					100

// When the camera's  is >= this position (looking down as we approach the VISION TETRA), we set the tilt servo to FINE_TRACKING and start
// to slowdown. The bigger the number, the closer to the target we get before slowing down.
		#define TILT_SERVO_FINE_SWITCH  	165

// When the camera's tilt servo is > this position (looking down as we approach the target), we are at the target.
// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_BEFORE_POSITION	180

// When the camera's tilt servo is > this position (looking down as we approach the VISION TETRA), we are at the vision tetra.
// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define	TILT_SERVO_AT_TETRA			200

// This should be the value of the cam.tilt_servo when the arm is lifted with the vision tetra attached. The lower the number the higher up.
		#define TILT_SERVO_TETRA_UP			170
		
// When the camera's  is >= this position (looking down as we approach the GOAL), we set the tilt servo to FINE_TRACKING and start
// to slowdown. The bigger the number, the closer to the target we get before slowing down.
		#define TILT_SERVO_FINE_GOAL_SWITCH 165

// When the camera's tilt servo is > this position (looking down as we approach the GOAL), we are at the GOAL.
// The bigger the number, the closer to the target we get before stopping and picking up vision tetra.
		#define TILT_SERVO_AT_GOAL      	190



#define	TETRA_LOAD_HEIGHT		5		// Height of tower to line up with vision tetra
#define	TETRA_UP_HEIGHT			20		// Height of tower to pick up tetra
#define	GOAL_UP_HEIGHT			40		// Height of tower to top of goal
#define	GOAL_DOWN_HEIGHT		30		// Height of tower to lower vision tetra



// Defines for states of the Autonomous State Machine positioning to loading station
		#define LOADING_STILL			1
		#define LOADING_FORWARD			2
		#define LOADING_FORWARD_DELAY	3
		#define LOADING_TURN_LEFT		4
		#define LOADING_TURN_RIGHT		5
		#define LOADING_FINAL_APPROACH	6
		#define LOADING_AT_TARGET		7



// Defines for different autonomous modes
#define AUTONOMOUS_LOAD 	1
#define GO_FOR_VISION_TETRA	2


// function prototypes
void MORT_autonomous_mode( int mode );
void load_goal(void);
void go_for_vision_tetra(void);

