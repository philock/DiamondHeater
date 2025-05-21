#ifndef STATUSINDICATOR_H
#define STATUSINDICATOR_H
#include <config.h>
#include <led.h>

class StatusIndicator{
    private:
        LED ledFault;
        LED ledOC;
        LED ledOT;
        LED ledActive;
        LED ledPower;

        void initLEDs();

    public:
        StatusIndicator();

        // Set status indicator LEDs
        void setFault(bool state)           { state ? ledFault.on() : ledFault.off(); }
        void setOverCurrent(bool state)     { state ? ledOC.on() : ledOC.off(); }
        void setOverTemperature(bool state) { state ? ledOT.on() : ledOT.off(); }
        void setActive(bool state)          { state ? ledActive.on() : ledActive.off(); }
        void setActiveBlink()               { ledActive.blink(); }
        void setPower(bool state)           { state ? ledPower.on() : ledPower.off(); }

        // Turn off all status indicator LEDs
        void reset();

        // Called continuously to refresh LEDs
        void update();

        // Status LEDs, specified by one byte: [x, x, x, x, Fault, OC, OT, Active] (MSB first)
        int getStatusWord();

};

#endif