 #include "PROJ100_Encoder.h"
#include <chrono>
#include "mbed_assert.h"
 

 /*
    Constructor
    Initialises relevant GPIOs and starts the event queue and timing thread as elevated priority threads
*/
PROJ100_Encoder::PROJ100_Encoder(PinName Output_Pin, uint16_t pulses_per_rotation) : _Encoder_Pin(Output_Pin){
    MBED_ASSERT(pulses_per_rotation > 0);
    _pulses_per_rotation = pulses_per_rotation;
    _stationary_millis = (uint16_t) (20000/pulses_per_rotation);
    _timing_thread.start(callback(this,&PROJ100_Encoder::timingUpdateThreadFunc));
    _timing_thread.set_priority(osPriorityAboveNormal);
    _stationary_thread.start(callback(&equeue, &EventQueue::dispatch_forever));
    _stationary_thread.set_priority(osPriorityAboveNormal);
}


 /*
    Private member function for setting the last pulse time
    This function is called using the timingUpdateThreadFunc thread
 */
void PROJ100_Encoder::setLastPulseTimeUs(uint32_t time){
    _last_pulse_time = time;
    _new_timing_data_available = true;   
}


/*
    Encoder GPIO interrupt handler - This runs in ISR context
    Reads the time from the pulse timer and update the last pulse time and sets isStaionary to false
*/
void PROJ100_Encoder::PROJ100_Encoder::encoderISR(){
    // if we've set a monitoring output pin to toggle from the interrupt, change its state
    if(out_pin && out_interrupt_or_thread ==OUTPUT_FROM_INTERRUPT){
        out_pin->write(!out_pin->read());
    }
    // Read the timer then reset it
    uint32_t tick_us = std::chrono::duration_cast<std::chrono::microseconds>(_pulse_timer.elapsed_time()).count();//was "_pulse_timer.read_us();"
    _pulse_timer.reset();

    // Place the data into the mailbox 
    uint32_t* p = timing_mail.try_alloc();
    if(p!=nullptr){
        *p = tick_us;
        timing_mail.put(p);
    }
}



uint16_t PROJ100_Encoder::getPulsesPerRotation(){
    return _pulses_per_rotation;
}

/*
    Elevated priority thread which waits from a mail object from the encoderISR and sets the pulse timing
*/
void PROJ100_Encoder::timingUpdateThreadFunc(){
    while(1){
        // Wait for a mail item with the timing data
        //following line to attempt deprecated .get() function i.e. newer try_get_for()..............................................MRS
        uint32_t* m = timing_mail.try_get_for(Kernel::wait_for_u32_forever);
        // Disable the stationary detection timer interrupt
        _stationary_timer.detach();
        // If we have mail (should always be there if we've unblocked from timing_mail.get(osWaitForever)
            // Extract the timing data and copy
            uint32_t time = *m;
            // Free the memory 
            timing_mail.free(m);
            // We need to update shared variables to aquire a mutex
            if(lock.try_acquire_for(20ms)){
                if(time > _debounce_time_us){
                    // Update the last pulse time
                   setLastPulseTimeUs(time);
                   // Set stationary detector to false (wheel is not stationary)
                   setIsStationary(false);
                    // if we've set a monitoring output pin to toggle from the thread, change its state
                    if(out_pin && out_interrupt_or_thread ==OUTPUT_FROM_THREAD){
                        out_pin->write(!out_pin->read());
                    }   
                }
                // Release the mutex
                lock.release();
            }
        // Reattach the stationary detection timer
        _stationary_timer.attach(callback(this,&PROJ100_Encoder::stationaryHandler),std::chrono::milliseconds(_stationary_millis));
    }
}


/*
    Handler for the stationary detector Timeout object.
    If there is more than _stationary_millis between ticks, this fires and informs that the wheel is stationary
    even when no pulse data is readable. The timeout is reset every time a pulse is received
*/
void PROJ100_Encoder::stationaryHandler(){
    equeue.call(this,&PROJ100_Encoder::setIsStationary,true);
    _stationary_timer.attach(callback(this,&PROJ100_Encoder::stationaryHandler),std::chrono::milliseconds(_stationary_millis));
}


/*
    Enables the encoder.
    Starts the pulse timer, enables the GPIO interrupt
    and starts the stationary detection timer.
    This must be called otherwise no data will be available
*/
void PROJ100_Encoder::start(){
    _pulse_timer.start();
    _Encoder_Pin.fall(callback(this,&PROJ100_Encoder::encoderISR));
    lock.release();
    _stationary_timer.attach(callback(this,&PROJ100_Encoder::stationaryHandler),std::chrono::milliseconds(_stationary_millis));
}


/*
    Disables the encoder
    Stops and resets the timers and removes all interrupts
*/
void PROJ100_Encoder::stop(){
    _Encoder_Pin.fall(NULL);
    _pulse_timer.stop();
    _pulse_timer.reset();
    _stationary_timer.detach();
    _new_timing_data_available = false;
    timing_mail.empty();
}

void PROJ100_Encoder::reset(){
    stop();
    start();
}


/*
    Public member function for setting the debounce time (in us)
    This helps to remove noise from false detection
*/
bool PROJ100_Encoder::setDebounceTimeUs(uint32_t new_us){
    if(lock.try_acquire_for(10ms)){
        _debounce_time_us = new_us;
        lock.release();
        return 1;
    }
    else{
        return 0;
    }
}


/*
    Public member function to return the current debounce time in us
*/    
uint32_t PROJ100_Encoder::getDebounceTimeUs(){
    if(lock.try_acquire_for(10ms)){
        bool dbt = _debounce_time_us;
        lock.release();
        return dbt;
    }
    else{
        return 0;
    }
}


/*
    Public non-blocking member function to return the last measured pulse length in us.
    Contains thread sychronisation to protects shared resources
    Once a valid value has been read and no new data is available, the fucntion will return 0
    If the wheel is staitonary the function will return -2
    If the function fails to aquire the semaphore, it returns -1
*/
int32_t PROJ100_Encoder::getLastPulseTimeUs(){
    int32_t rval=-1;
    if(lock.try_acquire_for(10ms)){
        if(_is_stationary){
            rval = -2;
        }
        if(!_new_timing_data_available){
            rval = 0;
        }
        else{
            rval = _last_pulse_time;
            _new_timing_data_available=false;
        }
        lock.release();
    }
    return rval;
}

/*
    Public non-blocking function to poll whether or not a pulse has been received 
    since the last time the function was called.
    returns -2 if the wheel is stationary, 0 if no pulse has been recieved but the wheel is
    not stationary or 1 if a pulse has been received
*/
int8_t PROJ100_Encoder::pulseReceived(){
    int32_t rval=-1;
    if(lock.try_acquire_for(10ms)){
        if(_is_stationary){
            rval = -2;
        }
        if(!_new_timing_data_available){
            rval = 0;
        }
        else{
            rval = 1;
            _new_timing_data_available=false;
        }
        lock.release();
    }
    return rval;
}

/*
    Public blocking function which will return 1 when a pulse is received
    an argument with a timeout can be passed and the function returns 0
    if a timeout occurs. If no timeout is passed, the default is osWaitforever 
    (will block forever or until pulse is recieved)
*/
bool PROJ100_Encoder::waitForPulse(uint32_t timeout_ms){
    Timer t;
    t.start();
    while(1){
        if(std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count() > timeout_ms){ // Was "t.read_ms()> timeout_ms"
            return false;
        }
        if(pulseReceived()==1){
            return true;
        }
    }
}


/*
    Public member function to poll whether or not the wheel is stationary
    by checking the _is_Staionary variable (set in the stationary detection timer ISR)
    and cleared when a pulse is received
*/
bool PROJ100_Encoder::isStationary(){
    if(lock.try_acquire_for(20ms)){
        bool stat = _is_stationary;
        lock.release();
        return stat;
    }
    else{
        //ToDo: Handle Error
        return false;
    }
}


/*
    Private function which sets whether or not the wheel is stationary
*/
void PROJ100_Encoder::setIsStationary(bool stat){
    _is_stationary = stat;
}

/*
    A DigitalOut can be used to monitor the pulses read by the MCU.
    This can be at the point of interrupt, or during the thread after debounce
*/
void PROJ100_Encoder::setOutputPin(PinName pin, uint8_t pos){
    if(out_pin){
        delete out_pin;
    }
    out_pin = new DigitalOut(pin);
    setOutputInterruptOrThread(pos);
}

/*
    Removes the DigitalOut object used for pulse read monitoring
    Called if the output pin is changed or removed and during deconstruction
*/
void PROJ100_Encoder::removeOutputPin(){
    if(out_pin){
        delete out_pin;
        out_pin=nullptr;
    }
}

/*
    Public member function for setting whether the output pulse is at the time of interrupt
    or during the thread after debounce. Values other than 0(OUTPUT_FROM_INTERRUPT) or 1(OUTPUT_FROM_THREAD) are ignoreed
*/
void PROJ100_Encoder::setOutputInterruptOrThread(uint8_t pos){
    if(pos==OUTPUT_FROM_INTERRUPT || pos==OUTPUT_FROM_THREAD){
        out_interrupt_or_thread = pos;
    }
}

/*
    Destructor deletes the digitalout pointer
*/
PROJ100_Encoder::~PROJ100_Encoder(){
    removeOutputPin();
}

