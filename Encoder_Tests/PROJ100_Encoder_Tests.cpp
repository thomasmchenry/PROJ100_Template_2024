#include "mbed.h"
#include "PROJ100_Encoder.h"
#include "PROJ100_Encoder_Tests.h"
#include "motor.h"
#include <chrono>
#include <cstdio>
#include <cwchar>

// Objects from main
extern PROJ100_Encoder left_encoder;
extern PROJ100_Encoder right_encoder;
extern Motor Wheel;

// Shared variables
float speed_setpoints[2] = {25.0f,30.0f}; //rpm
float dist_setpoints[2] = {2.0f,3.0f}; //rpm
float speed_setpoint = 30.0f;
float prev_setpoint=0.0f;
float last_rpm;

extern UnbufferedSerial ser;

Thread h_input_thread;

void speed_input_thread(){
    while(1){
        int le =0;
        int ri = 0;
        scanf("%d,%d",&le,&ri);
        speed_setpoints[0] = (float) le;
        speed_setpoints[1] = (float) ri;
    }
}

void distance_input_thread(){
    while(1){
        int le =0;
        int ri = 0;
        scanf("%d,%d",&le,&ri);
        dist_setpoints[0] = (float) le;
        dist_setpoints[1] = (float) ri;
    }
}


//typedef enum {TEST_TYPE_SPEED,TEST_TYPE_DISTANCE} test_type_t;
void start_input_thread(test_type_t type){
    switch(type){
        case TEST_TYPE_SPEED:{
            h_input_thread.start(speed_input_thread);
            break;
        }
        
        case TEST_TYPE_DISTANCE:{
            h_input_thread.start(distance_input_thread);
            break;
        }
    }
}

//*************************************Simple test***********************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses non-blocking PROJ100_Encoder::pulseReceived() function which returns 1 if a pulse has been
// detected since the last call, 0 if no pulse has been detected since the last call, -1 if the 
// wheel is stationary and -2 if there is an error

void simple_test(uint8_t L_or_R){
    DigitalOut led(LED1);
    Timer t;
    t.start();
    uint8_t counter = 0;
    Wheel.Speed(0.5f,0.6f);
    PROJ100_Encoder* enc = L_or_R == ENC_LEFT ? &left_encoder : &right_encoder;
    while(1){
        if(enc->pulseReceived()>0){
            uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count();//was t.read_ms();
            t.reset();
            led=!led;
            // Only print every 10 pulses
            counter++;
            if(counter>=10){
                counter=0;
                printf("\033[2J\033[1;1H\r\n");     // Clear the terminal
                printf("Pulse time %dms\r\n",ms);   // Print the pulse time
            }
        }
    }
}

//*************************************Blocking test***********************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses blocking PROJ100_Encoder::waitForPulse() function which will block (active state) waiting for a pulse to 
// be detected, or for the timeout period to have elapsed. This test uses a timeout period of osWaitForever
void blocking_test(uint8_t L_or_R){
    DigitalOut led(LED1);
    Timer t;
    t.start();
    uint8_t counter = 0;
    Wheel.Speed(0.8f,0.8f);
    PROJ100_Encoder* enc = L_or_R == ENC_LEFT ? &left_encoder : &right_encoder;
    while(1){
        enc->waitForPulse();
        uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count();//was t.read_ms();
        t.reset();
        led=!led;
        counter++;
        if(counter>=10){
            counter=0;
            printf("\033[2J\033[1;1H\r\n");
            printf("Pulse time %dms\r\n",ms);
        }
    }
}

//***********************************Blocking with timeout test*****************************************//
// Uses the encoder to measure the time between pulses using a Timer object t.
// Uses blocking PROJ100_Encoder::waitForPulse() function which will block (active state) waiting for a pulse to 
// be detected, or for the timeout period to have elapsed. This test uses a timeout period of 2000ms and 
// prints the pulse time every 10 pulses
void blocking_with_timeout_test(uint8_t L_or_R){
    DigitalOut led(LED1);
    Timer t;
    t.start();
    uint8_t counter = 0;
    Wheel.Speed(0.8f,0.8f);
    PROJ100_Encoder* enc = L_or_R == ENC_LEFT ? &left_encoder : &right_encoder;

    while(1){
        if(enc->waitForPulse(2000)){
            uint32_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count();//was t.read_ms()
            t.reset();
            led=!led;
            counter++;
            if(counter>=10){
                counter=0;
                printf("\033[2J\033[1;1H\r\n");
                printf("Pulse time %dms\r\n",ms);
            }
        }
        else{
            printf("\033[2J\033[1;1H\r\n");
            printf("NO PULSE\r\n");
        }
    }
}


//*************************************Speed test***********************************************//
// Uses the encoder information to control the speed of the wheels and display it on the terminal
// The speed can be changed by entering 2 integer values seperated by a comma
// E.g "11,33" will set the left motor to 11rpm and the right motor to 33rpm

void speed_test(){
    #define BUFFER_SIZE 64
    uint8_t averaging_window = 3;
    uint32_t sum[2] = {0,0};
    float last_rpm[2];
    uint32_t buffer[BUFFER_SIZE][2];
    //memset(buffer,0,sizeof(buffer));   sting.h function!!!!
    //memset(buffer,0,sizeof(buffer));
    // Clear the buffer with zeros
    for (int j=0;j<2;j++){
        for (int i=0;i<BUFFER_SIZE;i++){buffer[i][j]=0;}
    }
    int buffer_write_idx[2] = {0,0};
    uint8_t current_enc=ENC_LEFT;
    Timer print_timer;
    print_timer.start();
    start_input_thread(TEST_TYPE_SPEED);

    while(1){

        PROJ100_Encoder& enc = (current_enc==ENC_LEFT) ? left_encoder : right_encoder;
        // Check to see if a pulse has been received and it's time in us
        int32_t time = enc.getLastPulseTimeUs();

        // If a pulse has been received
        if(time>0){
            // Calculate sliding window average
            // Add the new value to the buffer
            buffer[buffer_write_idx[current_enc]][current_enc] = time;
            // Add the new value to the sum
            sum[current_enc] += time;
            // Calculate index of oldest data
            uint16_t idx = (buffer_write_idx[current_enc]>=averaging_window) ? (buffer_write_idx[current_enc]-averaging_window) : (buffer_write_idx[current_enc]+BUFFER_SIZE-averaging_window);
            // Remove oldest data from the sum
            sum[current_enc] -= buffer[idx][current_enc];
            // Calculate average
            float average = (float)sum[current_enc]/(averaging_window);
            // Increment buffer write index
            buffer_write_idx[current_enc] = (buffer_write_idx[current_enc] >= (BUFFER_SIZE-1)) ? 0 : buffer_write_idx[current_enc]+1;

            // Calculate RPM
            int ppr = enc.getPulsesPerRotation();
            float rpm = (60000000.0f/(ppr*average));
            // Calculate error term
            float err = speed_setpoints[current_enc]-rpm;
            // Adjust PWM values to match target speed (based on error term)
            float c=0.002;
            if(abs(err)>10.0f){
                c = 0.08;
            }
            else if( abs(err)>5.0f){
                c = 0.02;
            }
            else if( abs(err)>3.0f){
                c = 0.010;
            }
            if(current_enc==ENC_LEFT){
                // If target is 0, stop
                if(speed_setpoints[ENC_LEFT]==0){
                    Wheel.Speed(Wheel.getSpeedRight(),0.0f);
                }
                // Otherwise increase or decrease PWM accordingly
                else{
                    if(rpm < (speed_setpoints[current_enc])){
                        Wheel.Speed(Wheel.getSpeedRight(),Wheel.getSpeedLeft()+c);
                    }
                    else if(rpm > (speed_setpoints[current_enc])){
                        Wheel.Speed(Wheel.getSpeedRight(),Wheel.getSpeedLeft()-c);
                    }
                }  
            }
            else if(current_enc==ENC_RIGHT){
                // If target is 0, stop
                if(speed_setpoints[ENC_LEFT]==0){
                    Wheel.Speed(0.0f,Wheel.getSpeedLeft());
                }
                // Otherwise increase or decrease PWM accordingly
                else{
                    if(rpm < speed_setpoints[current_enc]){
                        Wheel.Speed(Wheel.getSpeedRight()+c,Wheel.getSpeedLeft());
                    }
                    else if(rpm > speed_setpoints[current_enc]){
                        Wheel.Speed(Wheel.getSpeedRight()-c,Wheel.getSpeedLeft());
                    }
                }
            }
            //Update for printing
            last_rpm[current_enc] = rpm;
        }
 
        // Check to see if the wheel IS staionary
        else if((enc.isStationary())){
            last_rpm[current_enc] = 0.0f;
            // SHOULD the wheel be stationary? If not apply some power to turn the wheel and allow the encoder
            // to measure the RPM again
            if(current_enc==ENC_LEFT && speed_setpoints[ENC_LEFT] !=0.0f){
                Wheel.Speed(Wheel.getSpeedRight(),0.8f);
            }
            else if(current_enc==ENC_RIGHT && speed_setpoints[ENC_RIGHT] !=0.0f){
                Wheel.Speed(0.8f,Wheel.getSpeedLeft());
            }
        }
        
        // Time will only = -1 if there is an error
        else if(time==-1){
            last_rpm[current_enc] = -999.0f;
        } 
        

        // Print periodically
        if(std::chrono::duration_cast<std::chrono::milliseconds>(print_timer.elapsed_time()).count()>=500){ //Was print_timer.read_ms()
            printf("\033[2J\033[1;1H\r\n");
            printf("Enter setpoints (integer) (L/R) seperated by comma\r\n");
            printf("Setpoint:\t%4.3f\t\t%4.3frpm\r\n",speed_setpoints[0],speed_setpoints[1]);
            printf("Current:\t%4.3f\t\t%4.3frpm\r\n",last_rpm[0],last_rpm[1]);
            print_timer.reset();
        } 
        current_enc=!current_enc;
    }
}

//*************************************Distance test********************************************//
// Uses the encoder to measure the distance the wheel has travelled and display it on the terminal
// The target distance can be changed by entering 2 integer values seperated by a comma
// E.g "1,2" will set the left target to 1m and the right target to 2m
void distance_test(){
    #define WHEEL_DIAMETER_MM_L       45
    #define WHEEL_DIAMETER_M_L       WHEEL_DIAMETER_MM_L/1000.0f
    #define DISTANCE_PER_ROTATION_L   3.14f * WHEEL_DIAMETER_M_L
    #define WHEEL_DIAMETER_MM_R       45
    #define WHEEL_DIAMETER_M_R       WHEEL_DIAMETER_MM_R/1000.0f
    #define DISTANCE_PER_ROTATION_R   3.14f * WHEEL_DIAMETER_M_R

    uint8_t current_enc=ENC_LEFT;
    float last_distance[2];
    int32_t pulse_counter[2]={0,0};
    const float distance_per_pulse[2] = {DISTANCE_PER_ROTATION_L/left_encoder.getPulsesPerRotation(),DISTANCE_PER_ROTATION_R/right_encoder.getPulsesPerRotation()};
    float delta[2];

    Timer print_timer;
    print_timer.start();
    start_input_thread(TEST_TYPE_DISTANCE);
    Wheel.Speed(1.0f, 1.0f);

    while(1){

        PROJ100_Encoder& enc = (current_enc==ENC_LEFT) ? left_encoder : right_encoder;

        // Check to see if a pulse has been received and it's time in us
        int32_t time = enc.getLastPulseTimeUs();

        // If a pulse has been received
        if(time>0){
            // Check whether the wheel has moved forwards or backwards
            int change = current_enc == ENC_LEFT ? Wheel.getDirectionLeft() : Wheel.getDirectionRight();
            // Update the pulse count accordingly
            pulse_counter[current_enc] += change;
            // Calculate distance based on wheel circumference
            float distance = distance_per_pulse[current_enc] * pulse_counter[current_enc];
            last_distance[current_enc] = distance;
            // Calculate the error term
            delta[current_enc] = dist_setpoints[current_enc]-distance;
            // Stop if we have reached our target distance
            if((delta[current_enc])<=(distance_per_pulse[current_enc]/2)){
                if(current_enc==ENC_LEFT){
                    Wheel.Speed(Wheel.getSpeedRight(),0.0f);          
                }
                else{
                    Wheel.Speed(0.0f,Wheel.getSpeedLeft());
                }
            }
        }

        // no pulse received, check to see if the wheel IS stationary
        else if((enc.isStationary()) && (abs(delta[current_enc])>(distance_per_pulse[current_enc]/2))){
            // SHOULD the wheel be stationary? If not apply some power to turn the wheel and allow the encoder to receive pulses
            if(current_enc==ENC_LEFT){
                Wheel.Speed(Wheel.getSpeedRight(),0.25f);
            }
            else if(current_enc==ENC_RIGHT){
                Wheel.Speed(0.25f,Wheel.getSpeedLeft());
            }
        }
        
        // Time will only = -1 if there is an error
        else if(time==-1){
            //printf("%s ERROR\r\n", ((current_enc==ENC_LEFT) ? "Left" : "Right"));
        } 

        // Print info periodically
        if(std::chrono::duration_cast<std::chrono::milliseconds>(print_timer.elapsed_time()).count()>=500){ //this was "if(print_timer.read_ms()>=500){""
            printf("\033[2J\033[1;1H\r\n");
            printf("Enter dist_setpoint integers(L/R) seperated by comma\r\n");
            printf("Setpoint:\t%4.3f\t\t%4.3fm\r\n",dist_setpoints[0],dist_setpoints[1]);
            printf("Current:\t%4.3f\t\t%4.3fm\r\n",last_distance[0],last_distance[1]);
            print_timer.reset();
        } 
        // Switch between left and right encoders
        current_enc=!current_enc;
    }
}