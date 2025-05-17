#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <led.h>
#include <input.h>
#include <ADC.h>
#include <config.h>
#include <PID_v2.h>
#include <DACx311.h>


Input buttonReset(PIN_RESET, false, true);

LED ledFault(PIN_FAULT);
LED ledOC(PIN_OC);
LED ledOT(PIN_OT);
LED ledActive(PIN_ACTIVE);
LED ledPower(PIN_POWER);

Adafruit_MAX31865 thermo(PIN_CS, &SPI);
ADC *adc = new ADC();
DACxx11 dac;

PID_v2 pid(PIDKp, PIDKi, PIDKd, PID::Direction::Direct);

elapsedMillis systemTick;

void resetSystem();

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

void resetSystem(){
    Serial.println("Resetting...");
    ledPower.off();
    //USB1_USBCMD = 0; // disconnect USB
    //delay(50);       // time for USB hubs/ports to detect disconnect
    SCB_AIRCR = 0x05FA0004; // Application Interrupt and Reset Control Register
}

void overCurrentProtection(){
    if(adc->adc0->isComplete()){
        constexpr float convFactor = (3.3 / (4095 * I_GAIN * R_SENSE));

        // Read analog value from ADC and convert to current in amps
        int val = adc->adc0->analogRead(PIN_ADC);
        float current = val * convFactor; 

        if(current > I_MAX){
            ledOC.on();
        }
    }
    else{
        adc->adc0->startSingleRead(A1);
    }
}

void printThermoFaults(){
    uint8_t fault = thermo.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
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
        thermo.clearFault();
    }
}

void setCurrent(float I){
    float U = I_GAIN*R_SENSE*I;
    dac.setVoltage(U);
}

void setup() {
    initButton();
    initADCDAC();
    initPID();
    initLEDs();

    thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
    thermo.enable50Hz(true);

    Serial.begin(115200);

    ledActive.blink();
}

void loop() {
    buttonReset.poll();
    ledActive.poll();

    if(systemTick > 100){
        //overCurrentProtection();
        //Serial.println(thermo.temperature(R_NOMINAL, R_REF));
        systemTick = 0;
    }

    /* for(float i = 0; i <= 0.41; i += 0.05){
        setCurrent(i);
        delay(1500);
        Serial.println(adc->adc0->analogRead(PIN_ADC)*(3.3 / (4095 * I_GAIN * R_SENSE)));
    }
    for(float i = 0.4; i >= -0.01; i -= 0.05){
        setCurrent(i);
        Serial.println(adc->adc0->analogRead(PIN_ADC)*(3.3 / (4095 * I_GAIN * R_SENSE)));
        delay(1500);
    } */

    const float I[] = {1.0, 2.0, 3.0, 4.0};
    for(unsigned int i = 0; i < sizeof(I)/sizeof(float); i++){
        //Serial.println(I[i]);
        setCurrent(I[i]);
        delay(100);
        Serial.println(adc->adc0->analogRead(PIN_ADC)*(3.3 / (4095 * I_GAIN * R_SENSE)));
        delay(50);
        setCurrent(0);
        delay(15);
    }

}
