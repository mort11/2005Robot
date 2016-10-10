#ifndef __user_camera_h_
#define __user_camera_h_

#define MAX_BUF_SIZE        64
#define CAMERA_PORT         1

/* Default Trackable Colors */
#define RED     1
#define GREEN   2
#define BLUE    3
#define YELLOW  4
#define WHITE   5

/* Two possible color modes*/
#define RGB     0
#define YCrCb   1

/* Two possible AUTO servo modes */
#define COARSE_TRACKING	0
#define FINE_TRACKING	1
#define SUPERFINE_TRACKING	2
#define SUPER_COARSE_TRACKING	3
#define DISABLE 4

/* 
	This is the main struct definition that gets filled when camera_track_update
    is called.  The servo values are only updated if the servo mode is enabled.
*/
typedef struct
{
  unsigned int  x,y;
  unsigned int  x1,y1,x2,y2;
  unsigned int  count,conf,pan_servo,tilt_servo;
} cam_struct;


/* 
	These commands can be used by the user to control the camera.
	Look in camera.c for more detailed comments.
*/
int Camera_Initialized(void);
int camera_stop(void);
int camera_track_update(void);
int camera_find_color(int color);
int camera_auto_servo(int pan, int sel);
int camera_set_servos(int pan, int tilt);

/*
	These commands are used internally by the camera driver.
	They could be used by you, but be careful.  
	Read the CMUcam manual!
*/
int camera_const_cmd(rom const char *cmd_str);
int camera_buffer_cmd(unsigned char *cmd_str);
int camera_set_exposure( int exposure );
int camera_reset(void);
int camera_getACK(void);
int wait_for_data(void);
void reset_rx_buffer(void);
void write_int_to_buffer(unsigned char *buf, int val );
void stop_streaming(void );
void restart_streaming(void);
void find_exposure( int color );
int get_tilt_tracking(void);
int camera_set_tilt( int tilt );
int camera_set_pan( int pan );

#endif
