#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

#include <stdint.h>
#include <stdbool.h>

struct PID {

 //Constants used in some of the functions below
 #define AUTOMATIC	1
 #define MANUAL	0
 #define DIRECT  0
 #define REVERSE  1
 #define P_ON_M 0
 #define P_ON_E 1

	
	double disp_kp;				// * we'll hold on to the tuning parameters in user-entered 
	double disp_ki;				//   format for display purposes
	double disp_kd;				//
	
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controller_direction;
	int pOn;

    double *my_input;              // * Pointers to the Input, Output, and Setpoint variables
    double *my_output;             //   This creates a hard link between the variables and the 
    double *my_setpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long last_time;
	double output_sum, last_input;

	unsigned long sample_time;
	double out_min, out_max;
	bool in_auto, pOnE;

};

void initialize(struct PID* pid);
void set_output_limits(double min, double max, struct PID* pid);
void set_mode(int mode, struct PID* pid);
void set_sample_time(int new_sample_time, struct PID* pid);
void set_controller_direction(int direction, struct PID* pid);
void set_tunings(double kp, double ki, double kd, int pon, struct PID* pid);
void init_pid(double* input, double* output, double* setpoint, double kp, double ki, double kd, int pon, int controller_direction, struct PID* pid);

#endif
