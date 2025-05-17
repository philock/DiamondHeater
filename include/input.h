#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>


#define INPUT_DEBOUNCE_MS 50


class Input{
    private:
        typedef void (*CallbackFunction)();
        CallbackFunction _activationHandler   = nullptr;
        CallbackFunction _deactivationHandler = nullptr;
        CallbackFunction _longpressHandler    = nullptr;

        int _pin;
        bool _activeHigh;

        bool _isRateLimited = false;
        int _rateLimitMS;

        int _debounceTimeMS = 50;
        int _longPressTimeMS = 1000;

        unsigned long _tNextRead;
        unsigned long _tLastChange;
        unsigned long _tRegisterLongpress;

        bool _isActive = false;
        bool _isLongpress = false;

        void handleInput();

    public:
        Input(int pin, bool activeHigh, bool inputPullup);

        void limitRate(int rateMS);
        void setDebounceTime(int tMS);
        void setLongpressTime(int tMS);

        void setActivationHandler(CallbackFunction f);
        void setDeactivationHandler(CallbackFunction f);
        void setLongpressHandler(CallbackFunction f);

        bool read();
        bool readLongpress();

        void poll();

};

#endif