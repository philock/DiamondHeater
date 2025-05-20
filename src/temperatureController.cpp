#include <temperatureController.h>

TemperatureController::TemperatureController():
    thermo(PIN_CS, &SPI),
    pid(PIDKp, PIDKi, PIDKd, PID::Direction::Direct),
    indicator(){

    initADCDAC();

    // Initialize MAX31865 thermo sensor
    thermo.begin(MAX31865_4WIRE);  // 4-wire resistance measurement
    thermo.enable50Hz(true);

    // Initialize PID
    pid.SetOutputLimits(0, I_MAX);    // Output between 0 and 5 amps
    pid.SetTunings(PIDKp, PIDKi, PIDKd);
    pid.SetSampleTime(PERIOD_CONTROLLER);
    pid.Start(_temperature,           // input
              0,                      // output
              _setpoint);             // setpoint
}

void TemperatureController::initADCDAC(){
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

void TemperatureController::setKp(float Kp){
    pid.SetTunings(Kp, pid.GetKi(), pid.GetKd());
}

void TemperatureController::setKi(float Ki){
    pid.SetTunings(pid.GetKp(), Ki, pid.GetKd());
}

void TemperatureController::setKd(float Kd){
    pid.SetTunings(pid.GetKp(), pid.GetKi(), Kd);
}

bool TemperatureController::setpoint(float T){
    if(T < 0 || T > T_MAX) return false;
    pid.Setpoint(T);
    return true;
}

bool TemperatureController::setCurrent(float I){
    if(I < 0 || I > I_MAX) return false;

    float U = I_GAIN*R_SENSE*I;
    dac.setVoltage(U);
    return true;
}

void TemperatureController::readCurrentTemperature(){
    _temperature = thermo.temperature(R_NOMINAL, R_REF);

    if (!adc->adc0->isComplete()) {
        adc->adc0->startSingleRead(PIN_ADC);
        return;
    }
    
    constexpr float convFactor = (3.3 / (4095 * I_GAIN * R_SENSE));
    int val = adc->adc0->analogRead(PIN_ADC);
    _current = val * convFactor;
}

void TemperatureController::protection(){
    // Over-current protection
    if(_current > I_MAX){
        indicator.setOverCurrent(true);
        error();
    }

    // Over-temperature protection
    if(_temperature > T_MAX){
        indicator.setOverTemperature(true);
        error();
    }

    // Thermometer errors
    uint8_t fault = thermo.readFault();
    if (fault){
        indicator.setFault(true);
        error();
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

void TemperatureController::updatePID(){
    float current_output = pid.Run(_temperature);
    setCurrent(current_output);
}

void TemperatureController::update(){
    readCurrentTemperature();

    if(_state == CtrlStates::START){       
        protection();
        updatePID();
    }
}

bool TemperatureController::start(){
    // Trun on PID
    pid.SetMode(PID::Mode::Automatic);

    // Blink "Active" led
    indicator.setActiveBlink();

    _state = CtrlStates::START;
}

bool TemperatureController::stop(){
    // Trun off PID and set output to zero
    pid.SetMode(PID::Mode::Manual);
    setCurrent(0.0);

    // "Active" LED on constant
    indicator.setActive(true);

    _state = CtrlStates::STOP;
}

void TemperatureController::error(){
    // Trun off PID and set output to zero
    pid.SetMode(PID::Mode::Manual);
    setCurrent(0.0);

    // "Active" LED off
    indicator.setActive(false);

    _state = CtrlStates::ERROR;
}

void TemperatureController::reset(){
    // reset indicator LEDs
    indicator.setFault(false);
    indicator.setOverCurrent(false);
    indicator.setOverTemperature(false);

    // Blink "Power" LED to indicate reset
    indicator.setPower(false);
    delay(500);
    indicator.setPower(true);

    // enter stop state
    stop();
}