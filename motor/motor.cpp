/** A Motor Driver, used for setting the pins for PWM Outpu
   *  for use with PROJ100
   *
   * @note Synchronization level: Interrupt safe
   *
   * Example:
   * @code
   * // Toggle a LED
   * #include "mbed.h"
   *
   * Motor Wheels(D15,D14,D13,D12);
   *
   * int main()
   * {    
   *     Wheel.Period_in_ms(2);//Set frequency of the PWMs 500Hz
   *     while(true)
   *     {
   *         Wheel.Speed(0.8,0.8);//Forward 80%
   *         wait(5.0);
   *         Wheel.stop();
   *         wait(1.0);
   *         Wheel.Speed(-0.8,-0.8);//Reverse 80%
   *         wait(5.0);
   *         Wheel.stop();
   *         wait(1.0);
   *     }
   * }
   * @endcode
*/

#include "motor.h"
Motor::Motor(PinName pinName1, PinName pinName2, PinName pinName3, PinName pinName4) : pin1(pinName1), pin2(pinName2), pin3(pinName3), pin4(pinName4)
{
}
void Motor::Fwd(float duty)
{ 
    this->pin1 = 0.0f;
    this->pin2 = duty;
    this->pin3 = 0.0f;
    this->pin4 = duty;
}
void Motor::Rev(float duty)
{
    this->pin1 = duty;
    this->pin2 = 0.0f;
    this->pin3 = duty;
    this->pin4 = 0.0f;
}
void Motor::Stop(void)
{
    this->pin1 = 0.0f;
    this->pin2 = 0.0f;
    this->pin3 = 0.0f;
    this->pin4 = 0.0f;
}
int Motor::Speed(float speedA, float speedB)
{
  if(speedA>1.0f||speedA<-1.0f){ //CHECK speedA Value is in Range!
      return -1;                  //return ERROR code -1=speedA Value out of range! EXIT Function
   }
   if(speedB>1.0f||speedA<-1.0f){ //CHECK speedB Value is in Range!
      return -2;                  //return ERROR code -2=speedB Value out of range! EXIT Function
   }
 
   //If speed values have passed the checks above then the following code will be executed
   
   if(speedA<0.0f)
   { //Reverse A motor
        this->pin1 = -speedA;
        this->pin2 = 0.0f;
   }
   
   else
   { //Forward A motor
        this->pin1 = 0.0f;
        this->pin2 = speedA;
   }
   
   if(speedB<0.0f)
   { //Reverse B motor
        this->pin3 = -speedB;
        this->pin4 = 0.0f;
   }
   else
   { //Forward B motor
        this->pin3 = 0.0f;
        this->pin4 = speedB;
   }
   return 0;                      //Return ERROR code Zero i.e. NO ERROR success!
}

void Motor::Period_in_ms(int msPeriod)
{
    this->pin1.period_ms(msPeriod);
    this->pin2.period_ms(msPeriod);
    this->pin3.period_ms(msPeriod);
    this->pin4.period_ms(msPeriod);
}

float Motor::getSpeedRight(){
    return (pin2.read() !=0.0f) ? pin2.read() : (pin1.read()*-1.0f);
}
float Motor::getSpeedLeft(){
    return (pin4.read() !=0.0f) ? pin4.read() : (pin3.read()*-1.0f);
}

int8_t Motor::getDirectionRight(){
    if(pin2.read() > 0.0f){
        return 1;
    }
    else if( pin1.read() >0.0f){
        return -1;
    }
    else return 0;
}

int8_t Motor::getDirectionLeft(){
    if(pin4.read() > 0.0f){
        return 1;
    }
    else if( pin3.read() >0.0f){
        return -1;
    }
    else return 0;
}
