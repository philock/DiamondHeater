#include <Arduino.h>

#define LED_TIME_BASEUNIT 200

enum LEDStates{
    ON, 
    OFF, 
    BLINK,
    ERR
};

class LED{
    public:
        LED(int pin);

        void on();
        void off();

        void blink();

        void errCode(const bool* morsePattern, int length);

        void poll();

    private:
        LEDStates _state = OFF;

        void stateBlink();
        void stateErr();

        int _pin;
        bool _ledOn;
        unsigned long _intervalEnd;

        const bool* _morsePattern;
        int _length;
        int _step;

};