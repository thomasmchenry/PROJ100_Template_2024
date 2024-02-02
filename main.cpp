#include "mbed.h"
#include "motor.h"
#include "tunes.h"
#include "pwm_tone.h"
#include "PROJ100_Encoder.h"
#include "PROJ100_Encoder_Tests.h"

#define TIME_PERIOD 7             //Constant compiler Values here 2 equates to 2ms or 500Hz base Frequency

#define ENCODER_PIN_LEFT            D8 
#define ENCODER_PIN_RIGHT           D6 

#define PULSES_PER_ROTATION_LEFT    20
#define PULSES_PER_ROTATION_RIGHT   20
#define DEBOUNCE_US                 30000

DigitalIn microswitch1(D4);       //Instance of the DigitalIn class called 'microswitch1'
DigitalIn microswitch2(D3);       //Instance of the DigitalIn class called 'microswitch2'

DigitalIn myButton(USER_BUTTON);
DigitalOut greenLED(LED1);

Motor Wheel(D9,D10,D13,D11);      //Instance of the Motor Class called 'Wheel' see motor.h and motor.cpp

PROJ100_Encoder right_encoder (ENCODER_PIN_LEFT,PULSES_PER_ROTATION_LEFT);
PROJ100_Encoder left_encoder(ENCODER_PIN_RIGHT,PULSES_PER_ROTATION_RIGHT);

UnbufferedSerial ser(USBTX,USBRX,115200);

//
int main ()
{
                printf("\033[2J\033[1;1H\r\n");
    printf("Plymouth University PROJ100 2023/24 Optical Encoder Demonstration\n\r");

    Wheel.Period_in_ms(TIME_PERIOD);//Set frequency of the PWMs

    right_encoder.stop();
    left_encoder.stop();

    while (myButton == 0){greenLED = !greenLED; ThisThread::sleep_for(100ms);}

    Wheel.Stop();
    ThisThread::sleep_for(1000ms);


    left_encoder.setOutputPin(D0);
    right_encoder.setOutputPin(D1);
    left_encoder.setOutputInterruptOrThread(OUTPUT_FROM_THREAD);
    right_encoder.setOutputInterruptOrThread(OUTPUT_FROM_THREAD);
    left_encoder.setDebounceTimeUs(DEBOUNCE_US);
    right_encoder.setDebounceTimeUs(DEBOUNCE_US);
    left_encoder.start();
    right_encoder.start();

    Wheel.Speed(0.8,0.8);

    /*********************ENCODER TESTS***************/
    // simple_test(ENC_LEFT);
    // blocking_test(ENC_LEFT);
    // blocking_with_timeout_test(ENC_LEFT);
     speed_test();
    // distance_test();

    while(true){};
}