#ifndef TEMPCTRL_H
#define TEMPCTRL_H
#include <Adafruit_MAX31865.h>
#include <DACx311.h>
#include <ADC.h>
#include <PID_v2.h>
#include <config.h>
#include <statusIndicator.h>

// State-definition for temperature controller finite state machine
enum CtrlStates{
    START,
    STOP,
    ERROR
};

class TemperatureController{
    private:
        Adafruit_MAX31865 thermo;
        ADC *adc = new ADC();
        DACxx11 dac;
        PID_v2 pid;
        
        float _setpoint =0;
        float _current =0;
        float _temperature =0;

        // State machine state
        CtrlStates _state = CtrlStates::STOP;
        
        // set current output (writes DAC)
        bool setCurrent(float I);

        // Initialize ADC and DAC settings
        void initADCDAC();

        // Run over-current and over-temperature protection
        void protection();

        // State entry function for error state
        void error();

        // Reads current and temperature and sets private variables
        void readCurrentTemperature();
        
        // Runs PID loop, sets DAC output
        void updatePID();
        
    public:
        StatusIndicator indicator;

        // Constructor
        TemperatureController();
        
        // Read actual temperature
        float getTemperature();

        // Read actual current
        float getCurrent();


        // Set target temperature
        bool setpoint(float T);

        
        // set PID loop gains
        void setKp(float Kp);
        void setKi(float Ki);
        void setKd(float Kd);


        // ---State entry functions---
        // Start system, regulate temperature 
        bool start();
        // Stop system, no output
        bool stop();
        // Resets controller to exit error state, into stop state: output and setpoint to zero, all indicator lights off.
        void reset();
        
        // ---Main loop function (FSM)---
        // Runs PID control loop, runs OT/OC protection, updates indicator lights
        void update();
        
};

#endif