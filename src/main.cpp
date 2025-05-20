#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <led.h>
#include <input.h>
#include <ADC.h>
#include <config.h>
#include <PID_v2.h>
#include <DACx311.h>
#include <comm.h>

// Input/output
Comm comm(115200);
Input buttonReset(PIN_RESET, false, true);
LED ledFault(PIN_FAULT);
LED ledOC(PIN_OC);
LED ledOT(PIN_OT);
LED ledActive(PIN_ACTIVE);
LED ledPower(PIN_POWER);

// Peripherals
Adafruit_MAX31865 thermo(PIN_CS, &SPI);
ADC *adc = new ADC();
DACxx11 dac;

PID_v2 pid(PIDKp, PIDKi, PIDKd, PID::Direction::Direct);
float setpoint = 0;

elapsedMillis systemTick;
elapsedMillis communicationTick;

// Some forward-declarations
void resetSystem();
void setCurrent(float I);

// ------------------------
// Initialization functions
// ------------------------
void initButton(){
    buttonReset.limitRate(100);
    buttonReset.setDebounceTime(200);
    buttonReset.setActivationHandler(resetSystem);
}

void initADCDAC(){
    // ADC configuration for high accuracy
    adc->adc0->setResolution(12);
    adc->adc0->setAveraging(32);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); 
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED); 

    adc->adc0->calibrate();
    adc->adc0->wait_for_cal();

    // initialize DAC
    dac.begin(PIN_SYNC, DAC7311, V_REF);
    dac.setVoltage(0);
}

void initLEDs(){
    const int T = 100; // LED test flash speed in ms

    ledFault.on();
    delay(T);

    ledOC.on();
    delay(T);

    ledOT.on();
    ledFault.off();
    delay(T);

    ledActive.on();
    ledOC.off();
    delay(T);

    ledPower.on();
    ledOT.off();
    delay(T);

    ledActive.off();
    delay(T);

    ledPower.off();
    delay(3*T);

    ledPower.on();
}

void initPID(){
    pid.SetOutputLimits(0, 4095); // Output limited by 12-bit DAC
    
}

// ------------------------
// System functions
// ------------------------

// Resets system to power-up state, erasing all errors
void resetSystem(){
    setpoint = 0;
    setCurrent(0);

    ledActive.off();
    ledFault.off();
    ledOC.off();
    ledOT.off();

    ledPower.off();
    delay(500);
    ledPower.on();

    //USB1_USBCMD = 0; // disconnect USB
    //delay(50);       // time for USB hubs/ports to detect disconnect
    //SCB_AIRCR = 0x05FA0004; // Application Interrupt and Reset Control Register
}

// Transmit state of fault, OT, OC and active LEDs back to host
void transmitStatusIndicators(){
    int state = 0;
    state |= (ledFault._ledOn << 3) | (ledOC._ledOn << 2) | (ledOT._ledOn << 1) | (ledActive._ledOn);
    comm.addVariableToken(state, MSG::STATUS);
}

// Transmit actual temperature
void transmitTemperature(){
    float t = thermo.temperature(R_NOMINAL, R_REF);
    comm.addVariableToken(t, MSG::T_ACTUAL);
}

// Run over-current and over-temperature protection
void protection(){
    // Over-current protection
    if(adc->adc0->isComplete()){
        constexpr float convFactor = (3.3 / (4095 * I_GAIN * R_SENSE));

        // Read analog value from ADC and convert to current in amps
        int val = adc->adc0->analogRead(PIN_ADC);
        float current = val * convFactor; 

        if(current > I_MAX){
            ledOC.on();
            transmitStatusIndicators();
        }
    }
    else{
        adc->adc0->startSingleRead(A1);
    }

    // Over-temperature protection
    if(thermo.temperature(R_NOMINAL, R_REF) > T_MAX){
        ledOT.on();
        transmitStatusIndicators();
    }

    // Thermometer errors
    uint8_t fault = thermo.readFault();
    if (fault){
        ledFault.on();
        /* Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
        }
        thermo.clearFault(); */
    }

}

// force-set current, without control loop
void setCurrent(float I){
    float U = I_GAIN*R_SENSE*I;
    dac.setVoltage(U);
}

// Read and process messages from host system
void handleSerial(){
    rx_message rxm = comm.getNextMsg();
    float payload;

    // Read messages until end of transmission
    while(rxm.msg != MSG::MSG_END){
        switch (rxm.msg){
            case MSG::PID_P:
                comm.getPayload(payload);
                pid.SetTunings(payload, pid.GetKi(), pid.GetKd());
                break;
            case MSG::PID_I:
                comm.getPayload(payload);
                pid.SetTunings(pid.GetKp(), payload, pid.GetKd());
                break;
            case MSG::PID_D:
                comm.getPayload(payload);
                pid.SetTunings(pid.GetKp(), pid.GetKi(), payload);
                break;
            case MSG::T_SETPOINT:
                comm.getPayload(payload);
                setpoint = payload;
            
            default:
                break;
        }
    }
}

void setup() {
    initButton();
    initADCDAC();
    initPID();
    initLEDs();

    thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo.enable50Hz(true);

    //Serial.begin(115200);

    ledActive.blink();
}

//float randomWalk = 0.0;
void loop() {
    buttonReset.poll();
    ledActive.poll();
    ledFault.poll();
    ledOC.poll();
    ledOT.poll();

    if(systemTick > 100){
        protection();
        handleSerial();
        //Serial.println(thermo.temperature(R_NOMINAL, R_REF));
        systemTick = 0;
    }
    if(communicationTick > 250){
        transmitStatusIndicators();
        transmitTemperature();
        comm.transmit();
        communicationTick = 0;
    }

    // Send random walk temperature for testing
    /* if(systemTick > 100){
        systemTick = 0;
        randomWalk += (float)random(-1000,1000)/1000.0;
        comm.addVariableToken(randomWalk, MSG::T_ACTUAL);
        comm.transmit();
    } */

}
