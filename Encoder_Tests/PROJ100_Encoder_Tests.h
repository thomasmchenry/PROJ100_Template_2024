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

void speed_input_thread();
void distance_input_thread();

typedef enum {TEST_TYPE_SPEED,TEST_TYPE_DISTANCE} test_type_t;
void start_input_thread(test_type_t);

//*************************************Simple test***********************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses non-blocking PROJ100_Encoder::pulseReceived() function which returns 1 if a pulse has been
// detected since the last call, 0 if no pulse has been detected since the last call, -1 if the 
// wheel is stationary and -2 if there is an error

void simple_test(uint8_t);

//*************************************Blocking test***********************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses blocking PROJ100_Encoder::waitForPulse() function which will block (active state) waiting for a pulse to 
// be detected, or for the timeout period to have elapsed. This test uses a timeout period of osWaitForever
void blocking_test(uint8_t);

//***********************************Blocking with timeout test*****************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses blocking PROJ100_Encoder::waitForPulse() function which will block (active state) waiting for a pulse to 
// be detected, or for the timeout period to have elapsed. This test uses a timeout period of 2000ms and 
// prints the pulse time every 10 pulses
void blocking_with_timeout_test(uint8_t);

//*************************************Speed test***********************************************//
// Uses the encoder information to control the speed of the wheels and display it on the terminal
// The speed can be changed by entering 2 integer values seperated by a comma
// E.g "11,33" will set the left motor to 11rpm and the right motor to 33rpm
void speed_test();

//*************************************Distance test********************************************//
// Uses the encoder to measure the distance the wheel has travelled and display it on the terminal
// The target distance can be changed by entering 2 integer values seperated by a comma
// E.g "1,2" will set the left target to 1m and the right target to 2m
void distance_test();

#endif