#ifndef _Turborobot_h_
#define _Turborobot_h_

//  The motor driver can handle a pwm frequency up to 20kHz
#define PWM_FREQUENCY 10000 
//  Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, 
//  we use no prescaling so frequency is 
//  given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
#define PWMVALUE F_CPU/PWM_FREQUENCY/2 // Max PWM value is 800

#define  forward 	1
#define  backward 	0

#define  motor_offset_left 	0
#define  motor_offset_right 	0

double lastError;                     // Store position error
double iTerm;                         // Store integral term
double roll, pitch;

double Kp = 0;
double Ki = 0;//3.8;
double Kd = 0;

double target_angle = 180.1 ;

unsigned char layingdown = 1;
#endif
