#include "../include/PID.h"

void pid_initialize(struct PID* pid){

    pid->output_sum = *(pid->my_output);
    pid->last_input = *(pid->my_input);
    if(pid->output_sum > pid->out_max) pid->output_sum = pid->out_max;
    else if(pid->output_sum < pid->out_min) pid->output_sum = pid->out_min;

}
void set_output_limits(double min, double max, struct PID* pid){
    
    if(min >= max) return;
    pid->out_min = min;
    pid->out_max = max;
    //omitted the if_auto statement because the PID will never be in auto.
    if(pid->in_auto){
        if(*(pid->my_output) > pid->out_max) *(pid->my_output) = (pid->out_max);
        else if(*(pid->my_output) < (pid->out_min)) *(pid->my_output) = pid->out_min;
    
    
        //ternary operator this. lol.
        if(pid->output_sum > pid->out_max) pid->output_sum = pid->out_max;
        else if(pid->output_sum < pid->out_min) pid->output_sum = pid->out_min;
    }


};

void set_mode(int mode, struct PID* pid){

    bool new_auto = (mode == AUTOMATIC);
    if(new_auto && !pid->in_auto){
        initialize(pid);
    }
    
    pid->in_auto = new_auto;
};

void set_sample_time(int new_sample_time, struct PID* pid){

    if(new_sample_time > 0){

        double ratio = (double)new_sample_time / (double)pid->sample_time;
        pid->ki *= ratio;
        pid->kd /= ratio;
        pid->sample_time = (uint64_t)new_sample_time;
        
    
    }

}; 

void set_controller_direction(int direction, struct PID* pid){

    pid->controller_direction = direction;
    
};

void set_tunings(double kp, double ki, double kd, int pon, struct PID* pid){

    if(kp < 0 || ki < 0 || kd < 0) return;

    pid->pOn = pon;
    pid->pOnE = pon == P_ON_E;
    pid->disp_kp = kp; pid->disp_kd = kd; pid->disp_ki = ki;
    
    
};

// * constructor.  links the PID to the Input, Output, and 
//   Setpoint.  Initial tuning parameters are also set here.
void init_pid(double* input, double* output, double* setpoint, double kp, double ki, double kd,       int pon, int controller_direction, struct PID* pid){

    pid->my_output = output;
    pid->my_input = input;
    pid->my_setpoint = setpoint;
    pid->in_auto = 1;
    pid->sample_time = 100;
    
    set_controller_direction(controller_direction, pid);


};

