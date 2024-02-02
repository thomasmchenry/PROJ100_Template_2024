#ifndef PROJ100_ENCODER_TESTS_H
#define PROJ100_ENCODER_TESTS_H
#include "mbed.h"
#include "PROJ100_Encoder.h"
#include "motor.h"
#include <chrono>
#include <cstdio>
#include <cwchar>

// Objects from main
extern PROJ100_Encoder left_encoder;
extern PROJ100_Encoder right_encoder;
extern Motor Wheel;
extern UnbufferedSerial ser;



//*************************************Simple test***********************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses non-blocking PROJ100_Encoder::pulseReceived() function which returns 1 if a pulse has been
// detected since the last call, 0 if no pulse has been detected since the last call, -1 if the 
// wheel is stationary and -2 if there is an error
void simple_test();



//*************************************Speed test***********************************************//
// Uses the encoder information to control the speed of the wheels and display it on the terminal
// Uses the non-blocking 
void speed_test();



#endif