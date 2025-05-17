#include "input.h"

Input::Input(int pin, bool activeHigh, bool inputPullup){
    if(inputPullup) pinMode(pin, INPUT_PULLUP);
    else            pinMode(pin, INPUT);    
    
    _activeHigh = activeHigh;
    _pin        = pin;
}

void Input::limitRate(int rateMS){
    if(rateMS != 0){
        _isRateLimited = true;
        _rateLimitMS   = rateMS;

    }
    else{
        _isRateLimited = false;

    }
}

void Input::setDebounceTime(int tMS){
    _debounceTimeMS = tMS;
}

void Input::setLongpressTime(int tMS){
    _longPressTimeMS = tMS;
}


void Input::setActivationHandler(CallbackFunction f){
    _activationHandler = f;
}

void Input::setDeactivationHandler(CallbackFunction f){
    _deactivationHandler = f;
}

void Input::setLongpressHandler(CallbackFunction f){
    _longpressHandler = f;
}

bool Input::read(){
    unsigned long time = millis();

    if(_isRateLimited){

        if(time > _tNextRead){
            _tNextRead = time + _rateLimitMS;

            // invert input if active low
            bool state = _activeHigh == digitalRead(_pin);

            bool debounceDelay = time > _tLastChange + _debounceTimeMS;

            if(debounceDelay && _isActive != state){
                _isActive = state;
                _tLastChange = time;

            }
        }

    }
    else{
        // invert input if active low
        bool state = _activeHigh == digitalRead(_pin);

        bool debounceDelay = time > _tLastChange + _debounceTimeMS;

        if(debounceDelay && _isActive != state){
            _isActive = state;
            _tLastChange = time;

        }

    }

    return _isActive;
}

bool Input::readLongpress(){
    bool longpressDelay = millis() > _tLastChange + _longPressTimeMS;

    if(read() && longpressDelay){
        _isLongpress = true;

    }
    else{
        _isLongpress = false;
        
    }

    return _isLongpress;
}

void Input::poll(){
    if(_isRateLimited){

        if(millis() > _tNextRead){
            _tNextRead = millis() + _rateLimitMS;
            handleInput();

        }

    }
    else{
        handleInput();

    }
}

void Input::handleInput(){
    // invert input if active low
    bool state = _activeHigh == digitalRead(_pin);

    bool debounceDelay = millis() > _tLastChange + _debounceTimeMS;

    // rising edge detected, call handler function
    if(debounceDelay && _isActive == false && state == true){
        
        _isActive = true;
        _tLastChange = millis();
        _tRegisterLongpress = _tLastChange + _longPressTimeMS;

        if(_activationHandler) _activationHandler();

        return;
    }

    // falling edge detected, call handler function
    if(debounceDelay && _isActive == true && state == false){

        _isActive = false;
        _isLongpress = false;
        _tLastChange = millis();

        if(_deactivationHandler) _deactivationHandler();

        return;
    }

    bool longpressDelay = millis()  > _tRegisterLongpress;

    if(longpressDelay && _isLongpress == false && _isActive == true){

        _isLongpress = true;

        if(_longpressHandler) _longpressHandler();

    }
}