#include "led.h"

LED::LED(int pin){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    
    _state = LEDStates::OFF;
    _pin = pin;

}

void LED::on(){
    digitalWrite(_pin, HIGH);

    _ledOn = true;
    _state = LEDStates::ON;

}

void LED::off(){
    digitalWrite(_pin, LOW);

    _ledOn = false;
    _state = LEDStates::OFF;

}

void LED::blink(){
    _state = LEDStates::BLINK;

    digitalWrite(_pin, LOW);
    _ledOn = false;

}

void LED::errCode(const bool* morsePattern, int length){
    _morsePattern = morsePattern;
    _length = length;
    _step = 0;
    _intervalEnd = millis();

    digitalWrite(_pin, LOW);
    _ledOn = false;

    _state = LEDStates::ERR;
}

void LED::poll(){
    switch (_state){
    case LEDStates::BLINK:
        stateBlink();
        break;

    case LEDStates::ERR:
        stateErr();
        break;
    
    default:
        break;
    }
}

void LED::stateBlink(){
    if(millis() > _intervalEnd){
        _ledOn = !_ledOn;

        digitalWrite(_pin, _ledOn);

        _intervalEnd = millis() + LED_TIME_BASEUNIT;

    }
}

void LED::stateErr(){
    if(millis() > _intervalEnd){

        if(!_ledOn){ // led on next

            if(_morsePattern[_step]) _intervalEnd = millis() + 3*LED_TIME_BASEUNIT; // dash
            else                     _intervalEnd = millis() + LED_TIME_BASEUNIT;   // dot

            _step ++;

        }
        else{ // pause next

            if(_step == _length){ // end of code, long pause
                _intervalEnd = millis() + 8*LED_TIME_BASEUNIT;
                _step = 0;
                
            }
            else _intervalEnd = millis() + 2*LED_TIME_BASEUNIT; // short pause

        }

        _ledOn = !_ledOn;
        digitalWrite(_pin, _ledOn);
    }
}