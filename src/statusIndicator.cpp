#include <statusIndicator.h>

StatusIndicator::StatusIndicator() 
    : ledFault(PIN_FAULT), ledOC(PIN_OC), ledOT(PIN_OT), 
      ledActive(PIN_ACTIVE), ledPower(PIN_POWER) {
    initLEDs();
}

void StatusIndicator::initLEDs(){
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

void StatusIndicator::reset(){
    ledFault.off();
    ledOC.off();
    ledOT.off();
    ledPower.off();
    ledActive.off();
}

void StatusIndicator::update(){
    ledActive.poll(); // For LED blinking
}

int StatusIndicator::getStatusWord(){
    int state = 0;
    state |= (ledFault._ledOn << 3) | (ledOC._ledOn << 2) | (ledOT._ledOn << 1) | (ledActive._ledOn);
    return state;
}