/******************************************************************************************************
FILE NAME: user_camera.c

MODIFICATION HISTORY:
		11/28/04 First Version by Anthony Rowe

DESCRIPTION:
	This file contains a set of function that communicate with the CMUcam.  It also requires a 
	modification to the PicSerialdrv.c file that is standard with the FRC distribution. This 
	modification allows the uart interrupt routine to buffer the CMUcam packets.

LIMITATION:
  Sometimes, after pressing the reset button, the camera will not come up correctly.  If you 
  power cycle the camera when it is stuck - it will work correctly.  It seems to always work
  from an RC power up.

******************************************************************************************************/
#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_Serialdrv.h"
#include "user_camera.h"
#include "MORT2005.H"


extern unsigned int parse_mode;
extern volatile unsigned int cam_index_ptr;
extern volatile unsigned int data_rdy;
unsigned char cam_uart_buffer[64]; 
int calibrate_running = 0;
int cam_state_flag = 0;
int delay = 0;
int exposure_val = 0;
int	number_of_loops = 0;
int confidence_average = 0;
int pixel_average = 0;
int tilt_tracking;
int pan_tracking;
int best_confidence = 0;
int best_exposure 	= 0;
int	num_of_frames = 0;
int one_second_delay = 0;
int num_of_samples = 0;
int	display_x = 0;

cam_struct cam;   // Main camera struct that is defined in camera.h, use this to access all camera data



rom const char *track_red    = "TC 190 255 0 255 0 40";		// Track red color in YCrCb mode    (Default "TC 190 255 0 255 0 40")

rom const char *track_green  = "TC 85 120 0 255 80 150"; 	// Track green color in yCrCb mode  (Default "TC 85 120 0 255 80 150")

rom const char *track_blue   = "TC 0 150 0 150 70 255";		// Track blue color in RGB mode		(Default "TC 0 150 1 150 70 255")

rom const char *track_yellow = "TC 100 255 75 255 0 20";	// Track yellow color in RGB mode   (Default "TC 100 255 75 255 0 20")

rom const char *track_white  = "TC 150 255 150 255 50 255";	// Track yellow color in RGB mode   (Default "TC 150 255 150 255 50 255")
													

rom const char *noise_filter 	= "NF 8";  			// Set noise filter to build up a tolerance to stray pixels
rom const char *yCrCb_mode 		= "CR 18 0"; 		// Set into yCrCb mode instead of RGB
rom const char *RGB_mode 		= "CR 18 8";  		// Set RGB mode
rom const char *aec_disable 	= "CR 41 128";		// Disable automatic exposure 
rom const char *manual_agc 		= "CR 19 32";		// Enable manual gain setting 			
rom const char *raw_mode 		= "RM 1";			// Enable Raw mode for TC packets, not ASCII
rom const char *virtual_window 	= "VW 1 1 159 238";

/**********************************************************************
Camera_Initialized

This function resets the camera and checks if it's ready

It only returns 1 (TRUE) if initialization was sucessfull
**********************************************************************/
#define		CAMERA_WAIT		1
#define		CAMERA_RESET	2
#define		CAMERA_READY	3

int Camera_Initialized( void )
{
  	int r;

	switch (cam_state_flag)
  	{
	case 0:
    	delay = 38;              				// Wait for 1 second
      	cam_state_flag = 1;						// Go to case 1 next loop
      	return 0;								// Not ready yet

    case CAMERA_WAIT:
      	if (--delay == 0)						// If delay is done			
        	cam_state_flag = CAMERA_RESET;		// Try to reset camera
		return 0;								// Not ready yet		

    case CAMERA_RESET:
	  	if(camera_reset()==1) {					// If camera reset sucessfully
		  	r=0;								// Init ACK counter
		  	r+=camera_const_cmd(aec_disable ); 	// Disable AEC
		  	r+=camera_const_cmd(manual_agc ); 	// Enable manual AEC and AGC
		  	r+=camera_const_cmd(noise_filter ); // Set noise filter to 6 pixels
		  	r+=camera_const_cmd(raw_mode ); 	// Set camera output to BINARY
		  	if(r == 4) {
	      		camera_auto_servo( COARSE_TRACKING, COARSE_TRACKING );	// Turn on auto-servo mode
		 	 	Relay1_green=0; Relay2_green=0; Switch1_LED=0; Switch2_LED=0; Switch3_LED=0; 
				cam_state_flag = CAMERA_READY;				// Set state to "Ready" 
				printf( "Camera Initialization Sucessful...\r");
				return 0;						// We'll be ready next loop
			}
		}
    	printf( "\r\rCamera Initialization Failure...\r\r" );
		Relay1_green ^= 1,Relay2_green ^= 1,Switch1_LED  ^= 1,Switch2_LED ^= 1,Switch3_LED  ^= 1;
        cam_state_flag = 0;   					// Issue continual retries until camera responds
		return 0;								// Not ready yet		

	case CAMERA_READY:
        return 1;								// Return Success that camera is ready
  	}
}


int camera_find_color(int color)
{
  	int chk=0;
  
  	camera_stop();          						// Stop the camera if it is already tracking
  	chk+=camera_reset();      						// Reset the camera, and make sure it reset (actually check for true return)
	chk+=camera_const_cmd(noise_filter);			// Enable the same noise filter for all tracking
	chk+=camera_const_cmd(raw_mode);    			// Put into raw mode, this makes it send output numbers as bytes instead
	camera_auto_servo( COARSE_TRACKING, COARSE_TRACKING );		// Turn on auto-servo mode

  	switch(color)
  	{
  	case RED:
		camera_const_cmd(yCrCb_mode);				// Set yCrCb mode
		camera_set_exposure(FIRST_RED);				// Set manual exposure value
    	camera_const_cmd(track_red);				// Now start tracking RED
   		printf("Looking for RED\r");
    	break;

  	case GREEN:
		camera_const_cmd(yCrCb_mode);				// Set yCrCb mode
		camera_set_exposure(FIRST_GREEN);			// Set manual exposure value
    	camera_const_cmd(track_green);				// Now start tracking GREEN
   		printf("Looking for GREEN\r");
    	break;

  	case BLUE:   
		camera_const_cmd(RGB_mode);					// Set RGB mode
		camera_set_exposure(FIRST_BLUE);			// Set manual exposere value
		camera_const_cmd(track_blue);				// Now start tracking BLUE
   		printf("Looking for BLUE\r");
		break;

  	case YELLOW:   
		camera_const_cmd(RGB_mode);					// Set RGB mode
		camera_set_exposure(FIRST_YELLOW);			// Set manual exposere value
		camera_const_cmd(track_yellow);				// Now start tracking BLUE
   		printf("Looking for YELLOW\r");
		break;

  	case WHITE:   
		camera_const_cmd(RGB_mode);					// Set RGB mode
		camera_set_exposure(FIRST_WHITE);			// Set manual exposere value
		camera_const_cmd(track_white);				// Now start tracking WHITE
   		printf("Looking for WHITE\r");
		break;

  	default:
    	return 0;
  	}

	if( chk != 3 ) {
		printf( "\r\rOne of the settings in camera_find_color( ) failed\r\r" );
		return 0;
	}		

	parse_mode=1; 						// Now waiting for Track Color Packet in interrupt
  	return 1;
}





/**********************************************************************
camera_set_servos

	This function sets the pan and the tilt servo position.
	Note, that this function will stop the camera from tracking.
	If you are tilting the head to see if something is there, you need
	to send a fresh track command.  Maybe it would be more efficient to
	use the RC's servo output.
	
	int pan  - servo pan value from 46 to 210, with 128 as the middle
			 - Yes, we know that seems strange, but there is a good reason
	int tilt - servo tilt value from 46 to 210, with 128 as the middle

	Return: 0 	Success
			-1	Missing ACK, or timeout

**********************************************************************/
int camera_set_servos( int pan, int tilt )
{

	if( camera_set_pan( pan ))
		return -1;
	if( camera_set_tilt( tilt ))
		return -1;
	return 0;				// Return 0 for success
}


int camera_set_pan( int pan )
{
	unsigned char buf[32];
	buf[0]='S';
	buf[1]='V';
	buf[2]=' ';
	buf[3]='0';  // Servo 0 is the pan
	buf[4]=' ';
	write_int_to_buffer(&buf[5], pan );
  	if(camera_buffer_cmd(buf))
		return 0;				// Return 0 for Success
	else
		return -1;				//  Return -1 for error
}

int camera_set_tilt( int tilt )
{
	unsigned char buf[32];
	buf[0]='S';
	buf[1]='V';
	buf[2]=' ';
	buf[3]='1';  // Servo 1 is the tilt
	buf[4]=' ';
	write_int_to_buffer(&buf[5], tilt );
  	if(camera_buffer_cmd(buf))
		return 0;				// Return 0 for Success
	else
		return -1;				//  Return -1 for error
}


/**********************************************************************
write_int_to_buffer

This function takes a buffer and adds an ascii value to the end of it.
	unsigned char *buf - Location in buffer to start appending a value
	int val	- the integer value that is to be added to the buffer

**********************************************************************/
void write_int_to_buffer(unsigned char *buf, int val )
{
  int digit;
  int index;
  index=0;
  digit=val/100;
  digit=digit%10; // This grabs the hundreds place digit
  if(digit>0)    // If it is a zero, we don't need to print it
  {
    buf[index]=digit+48;
    index++;
  }
  digit=val/10;
  digit=digit%10;    // This grabs the tens place digit
  if(index>0 || digit>0 )  // If the hundreds place was a 0 and this is also
  {					// a 0, skip it, else print the number
    buf[index]=digit+48;
    index++;
  }
  digit=val%10;  // Print out the ones place digit no matter what
  buf[index]=digit+48;
  index++;
  buf[index]=0;  // Add a null character to terminate the string
  
}


void stop_streaming(void )
{
	camera_stop( );			// stop camera from streaming
}

void restart_streaming(void)
{
    camera_const_cmd( "TC" );  		// Restart tracking at last RGB values
	parse_mode = 1;
}

/********************************************************************************
camera_track_update

This function should be called after a color tracking call. The interrupt
will fill this with a T packet from the camera if parse_mode is set to 1.
parse_mode is set to 1 by track color.

	Return:  1 when new data is ready (only read packet when 1)
			 0 when waiting for the camera
*********************************************************************************/
int camera_track_update(void)
{
  int i;
  
  if(wait_for_data()==0) return 0;
  
  if(cam_uart_buffer[0]=='T' && cam_index_ptr>=9)
  {
    cam.x=cam_uart_buffer[1];
    cam.y=cam_uart_buffer[2];
    cam.x1=cam_uart_buffer[3];
    cam.y1=cam_uart_buffer[4];
    cam.x2=cam_uart_buffer[5];
    cam.y2=cam_uart_buffer[6];
    cam.count=cam_uart_buffer[7];
    cam.conf=cam_uart_buffer[8];
	if(cam_index_ptr>9)
    {
		cam.pan_servo=cam_uart_buffer[9];
      	cam.tilt_servo=cam_uart_buffer[10];
    }
    reset_rx_buffer();
    return 1;
  }
  // Bad data in buffer
  reset_rx_buffer();
  return 0;
}

/**********************************************************************
camera_configure

This function shows how you can set the exposure gain and color_mode
of the camera manually. 
	int exposure - sets the exposure level
				 - lower values make the image darker
	int color_mode - RGB puts the camera in RGB mode
				   - YCrCb puts the camera in YCrCb mode

    Return: 1 ACK
			0 no ACK, maybe a NCK or a timeout
**********************************************************************/
int camera_set_exposure( int exposure )
{
  unsigned char buf[32];
  int chk=0;
  
  chk+=camera_const_cmd( "CR 41 128" );		// Disable automatic AEC
  chk+=camera_const_cmd( "CR 19 32"  );		// Allow manual settings

  buf[0]='C';
  buf[1]='R';
  buf[2]=' ';
  buf[3]='1';
  buf[4]='6';
  buf[5]=' ';
  write_int_to_buffer(&buf[6], exposure ); 	// Send the constructed exposure
  chk+=camera_buffer_cmd( buf ); 

  chk+=camera_const_cmd( "CR 0 32" ); 		// Set AGC gain to mid level

  if(chk<4)
     printf( "\r\rOne of the camera settings in camera_configure failed\r\r" );
}


/**********************************************************************
camera_getACK

This function sends the final '\r' character and waits to see if the
camera returns an ACK or a NCK. This only works when parse_mode = 0 and
should be used for control commands to the camera, and not for tracking
commands.

    Return: 1 ACK
			0 no ACK, maybe a NCK or a timeout
**********************************************************************/
int camera_getACK(void)
{
  int cnt,i;
  Serial_Write(CAMERA_PORT,"\r",1);
  reset_rx_buffer();
  if( wait_for_data()==0) return 0;
  if(cam_uart_buffer[0]==65) 
  {
    reset_rx_buffer();
    return 1;
  }
  if(cam_uart_buffer[0]==84) 
  {
    return 2;
  }
  reset_rx_buffer();
  return 0;
  
}

/**********************************************************************
camera_auto_servo

This function turns on auto servo mode and sets the tracking to fine or coarse

	 Return: Nothing
**********************************************************************/
int camera_auto_servo(int pan, int tilt)
{
	unsigned char buf[32];
	int	chk= 0;
	
	pan_tracking = pan;
	tilt_tracking = tilt;

	printf("AUTO servo mode...PAN servo = ");

  	buf[0]='S';
  	buf[1]='P';
  	buf[2]=' ';
	if( pan_tracking == FINE_TRACKING ) {
		printf("FINE_TRACKING");
		buf[3]  ='0';
		buf[4]  ='8';
	  	buf[5]  =' ';
		buf[6]  ='0';
  		buf[7]  ='4';
	  	buf[8]  =' ';
  		buf[9]  ='2';
	  	buf[10] =' ';
	  	buf[11] = ' ';
	}
	if( pan_tracking == SUPERFINE_TRACKING ) {
		printf("SUPERFINE_TRACKING");
		buf[3]  ='0';
		buf[4]  ='1';
	  	buf[5]  =' ';
  		buf[6]  ='0';
  		buf[7]  ='1';
	  	buf[8]  =' ';
  		buf[9]  ='1';
	  	buf[10] =' ';
	  	buf[11] = ' ';
	}
	if( pan_tracking == SUPER_COARSE_TRACKING)  {
		printf("SUPER_COARSE_TRACKING");
		buf[3]  = '9';
		buf[4]  = '9';
	  	buf[5]  = ' ';
  		buf[6]  = '6';
  		buf[7]  = '4';
	  	buf[8]  = ' ';
  		buf[9]  = '3';
	  	buf[10] ='2';
	  	buf[11] = ' ';

	}
	if( pan_tracking == COARSE_TRACKING || pan_tracking == DISABLE)  {
		if(pan_tracking == COARSE_TRACKING)
			printf("COARSE_TRACKING");
		else
			printf("DISABLED");
		buf[3]  ='1';
		buf[4]  ='6';
	  	buf[5]  =' ';
  		buf[6]  ='1';
  		buf[7]  ='2';
	  	buf[8]  =' ';
  		buf[9]  ='4';
	  	buf[10] =' ';
	  	buf[11] = ' ';
	}

	printf("    TILT servo = ");
	if( tilt_tracking == FINE_TRACKING ) {
		printf("FINE_TRACKING");
		buf[12] ='0';
		buf[13] ='8';
	  	buf[14] =' ';
		buf[15] ='0';
  		buf[16] ='4';
	  	buf[17] =' ';
  		buf[18] =' ';
	  	buf[19] ='2';
	  	buf[20] = 0;
	}
	if( tilt_tracking == SUPERFINE_TRACKING ) {
		printf("SUPERFINE_TRACKING");
		buf[12] ='0';
		buf[13] ='1';
	  	buf[14] =' ';
  		buf[15] ='0';
  		buf[16] ='1';
	  	buf[17] =' ';
  		buf[18] =' ';
	  	buf[19] ='1';
	  	buf[20] = 0;
	}
	if( tilt_tracking == SUPER_COARSE_TRACKING)  {
		printf("SUPER_COARSE_TRACKING");
		buf[12] = '9';
		buf[13] = '9';
	  	buf[14] = ' ';
  		buf[15] = '6';
  		buf[16] = '4';
	  	buf[17] = ' ';
  		buf[18] = '3';
	  	buf[19] = '2';
	  	buf[20] = 0;
	}
	if( tilt_tracking == COARSE_TRACKING || tilt_tracking == DISABLE)  {
		if(tilt_tracking == COARSE_TRACKING)
			printf("COARSE_TRACKING");
		else
			printf("DISABLED");
		buf[11] ='1';
		buf[12] ='2';
	  	buf[13] =' ';
  		buf[14] ='0';
  		buf[15] ='8';
	  	buf[16] =' ';
  		buf[17] =' ';
	  	buf[18] ='4';
	  	buf[19] = 0;
	}
	printf("\r");

  	chk+=camera_buffer_cmd( buf ); 
    if(pan == DISABLE && tilt == DISABLE )
		chk+=camera_const_cmd( "SM 12" );				// Enable control on Pan and Tilt

    if(pan == DISABLE && tilt != DISABLE )
		chk+=camera_const_cmd( "SM 14" );				// Enable control on Pan and Tilt

    if(pan != DISABLE && tilt == DISABLE )
		chk+=camera_const_cmd( "SM 13" );				// Enable control on Pan and Tilt

    if(pan != DISABLE && tilt != DISABLE )
		chk+=camera_const_cmd( "SM 15" );				// Enable control on Pan and Tilt
	
if( chk != 2 )
    	printf( "\r\rFailure in camera_auto_servo( ) function...\r\r" );
}


//************************************************************************
// Returns the state of the tilt servo ( FINE_TRACKING or COARSE_TRACKING )
int get_tilt_tracking(void)
{
	return tilt_tracking;
}


//************************************************************************
// Returns the state of the pan servo ( FINE_TRACKING or COARSE_TRACKING )
int get_pan_tracking(void)
{
	return pan_tracking;
}


/**********************************************************************
camera_const_cmd

This function is used to send constant string commands to the camera.
This means you have to send something in quotes.  If you are sending from
a buffer, you must use camera_buffer_cmd.  See camera_find_color for examples.

		rom const char *cmd_str - This is a constant string to be sent to the camera
								- It does not require the '\r' at the end

		Return: 0 - no ACK, or timed out
				1 - ACK, the command is good
**********************************************************************/
int camera_const_cmd(rom const char *cmd_str)
{
  int i;
  int len;
  for (i=0; i<MAX_BUF_SIZE; i++ )
  {
    if (cmd_str[i]=='\r' || cmd_str[i]==0 )
    {
      len = i;
      //	cmd_str[i]=0;
      break;
    }
  }
  Serial_Write(CAMERA_PORT,cmd_str,len);
  return camera_getACK();
}

/**********************************************************************
camera_const_cmd

This function is used to send a string from memory to the camera. It is
camera_const_cmd's twin, but is used for arrays that you make yourself.

		unsigned char *cmd_str - This is a string to be sent to the camera
								- It does not require the '\r' at the end

		Return: 0 - no ACK, or timed out
				1 - ACK, the command is good
**********************************************************************/
int camera_buffer_cmd(unsigned char *cmd_str)
{
  int i;
  int len;
  for(i=0; i<MAX_BUF_SIZE; i++ )
  {
    if(cmd_str[i]=='\r' || cmd_str[i]==0 )
    {
      len=i;
      break;
    }
  }
  Serial_Write_Bufr(CAMERA_PORT,cmd_str,len);
  return camera_getACK();
}

/**********************************************************************
camera_stop

This function stops the camera from streaming.  This should be used when
you are in the middle of tracking, and want to send new control commands.

		Return: 0 - no ACK, or timed out and may not have stopped
				1 - ACK, the command is good, the camera stopped
**********************************************************************/
int camera_stop(void)
{
  parse_mode=0;
  camera_getACK();
  return camera_getACK();
}

/**********************************************************************
reset_rx_buffer

This function will reset the pointer index and the data_rdy flag that
is used by the interrupt in PicSerialdrv.c
This is called right before the interrupt routine is supposed to look
for new data.
**********************************************************************/
void reset_rx_buffer(void)
{
	cam_index_ptr=0;
	data_rdy=0;
}

/**********************************************************************
wait_for_data

This function checks to see if the serial buffer for the camera has new
data ready.  Depending on what mode it is in, it may wait for a full T
packet, or just a '\r' terminated line. It only waits a short period of
time, and then it returns 0 if no data is ready.  This short period of
time is just long enough to catch ACKs from messages.

		Return: 0 - no new data
				1 - new packet is ready
**********************************************************************/
int wait_for_data(void)
{
  int i;

  // This loop below is a counter that gives just enough time to catch
  // an ACK from a normal command.  
  for(i=0; i<20000; i++ )     
	  if(data_rdy!=0 ) return 1;
	
  return 0;
}

/**********************************************************************
camera_reset

This function resets the camera.  This will clear all register.  It then
checks to see if the camera is responding.

		Return: 0 - no ACK, and reset may not have happened
				1 - reset occured, and all is well
**********************************************************************/
int camera_reset(void)
{
  int i;
  parse_mode=0;
  Serial_Write(CAMERA_PORT,"rs\r",3);
  i=camera_getACK();
  return camera_getACK();
}



void find_exposure( int color )
{
  	if(calibrate_running == 0) {

		camera_stop();          				// Stop the camera if it is already tracking
	  	camera_reset();      					// Reset the camera, and make sure it reset (actually check for true return)
		camera_const_cmd(noise_filter);			// Enable the same noise filter for all tracking
		camera_const_cmd(raw_mode);    			// Put into raw mode, this makes it send output numbers as bytes instead
		camera_const_cmd(virtual_window);
		camera_set_servos( 128, 128 ); 					// position camera to look for tetra 
		calibrate_running = 1;
		display_x = 0;

		printf("Calibrating for ");
		switch(color) {
	  		case GREEN:
	   			printf("GREEN\r");
			  	camera_const_cmd(aec_disable ); 	// Disable AEC
			  	camera_const_cmd(manual_agc ); 	// Enable manual AEC and AGC
				camera_const_cmd(noise_filter ); // Set noise filter to 6 pixels
		  		camera_const_cmd(raw_mode ); 	// Set camera output to BINARY
				camera_const_cmd(virtual_window);			// Set virtual window
				camera_const_cmd(yCrCb_mode);		// Set yCrCb mode
				camera_set_exposure(0);			// Set manual exposure value
		    	camera_const_cmd(track_green);
				best_confidence = 0;
				best_exposure = 0;
				one_second_delay = 0;
			break;

	  		case RED:
	   			printf("RED\r");
			  	camera_const_cmd(aec_disable ); 	// Disable AEC
			  	camera_const_cmd(manual_agc ); 	// Enable manual AEC and AGC
				camera_const_cmd(noise_filter ); // Set noise filter to 6 pixels
		  		camera_const_cmd(raw_mode ); 	// Set camera output to BINARY
				camera_const_cmd(virtual_window);			// Set virtual window
				camera_const_cmd(yCrCb_mode);		// Set yCrCb mode
				camera_set_exposure(0);			// Set manual exposure value
		    	camera_const_cmd(track_red);
				best_confidence = 0;
				best_exposure = 0;
				one_second_delay = 0;
			break;
	
	  		case YELLOW:
	   			printf("YELLOW\r");
			  	camera_const_cmd(aec_disable ); 	// Disable AEC
			  	camera_const_cmd(manual_agc ); 	// Enable manual AEC and AGC
				camera_const_cmd(noise_filter ); // Set noise filter to 6 pixels
		  		camera_const_cmd(raw_mode ); 	// Set camera output to BINARY
				camera_const_cmd(virtual_window);			// Set virtual window
				camera_set_exposure(0);			// Set manual exposure value
		    	camera_const_cmd(track_yellow);
				best_confidence = 0;
				best_exposure = 0;
				one_second_delay = 0;
			break;

	  		case BLUE:
	   			printf("BLUE\r");
				camera_const_cmd(RGB_mode);		// Set yCrCb mode
				camera_set_exposure(0);			// Set manual exposure value
		    	camera_const_cmd(track_blue);
			break;
	
	  	default:
			return;
		}
	return;
	}


	if(display_x == 1) {
		printf("Camera X = %d\r", cam.x );
		return;
	}

	if(exposure_val == 255) {
		printf("Best Exposure is %d, Best Confidence is %d\r", best_exposure, best_confidence);
		camera_stop();
		camera_set_exposure( best_exposure );
		camera_auto_servo(SUPERFINE_TRACKING, SUPERFINE_TRACKING);	// Pan.servo = COARSE, Tilt still COARSE
    	camera_const_cmd("TC");
		parse_mode = 1;		
		display_x = 1;
	}
	
  	if(camera_track_update( )) 			// If we have a new frame
	{
		num_of_frames++;
//		printf("Exposure= %03d Pixel Count = %03d, Confidence =  %03d\r", exposure_val, cam.count, cam.conf);
		if( cam.count > 1 ) {
			num_of_samples++;
			confidence_average += cam.conf;
			pixel_average += cam.count;
			}	
		}
	
	if(one_second_delay++ >= 9) {
		one_second_delay = 0;
		pixel_average = pixel_average/num_of_samples;
		if(pixel_average < 0 ) pixel_average = 0;
		confidence_average = confidence_average/num_of_samples;
		if(confidence_average < 0 ) confidence_average = 0;

		printf("FPS= %03d Exposure = %03d, Pixel Average = %03d, Conf Average = %03d\r", num_of_frames * 4, exposure_val, pixel_average, confidence_average);
		if ((confidence_average/num_of_samples) > best_confidence ) {
			best_confidence = confidence_average/num_of_samples;
			best_exposure = exposure_val;
		}
		exposure_val++;
		num_of_samples = 0;
		num_of_frames= 0;
		confidence_average = 0;
		pixel_average = 0;
		camera_stop();
		camera_set_exposure( exposure_val );
    	camera_const_cmd("TC");
		parse_mode = 1;		
	}

}
