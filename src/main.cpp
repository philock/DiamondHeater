#include <Arduino.h>
#include <temperatureController.h>
#include <interface.h>
#include <input.h>

Input buttonReset(PIN_RESET, false, true);
TemperatureController controller;
Interface interface(&controller);

elapsedMillis interfaceTick;
elapsedMillis controllerTick;

// Soft-resets system to power-up state, erasing all errors
void resetSystem(){
    controller.reset();

    // Microcontroller hard-reset
    //USB1_USBCMD = 0; // disconnect USB
    //delay(50);       // time for USB hubs/ports to detect disconnect
    //SCB_AIRCR = 0x05FA0004; // Application Interrupt and Reset Control Register
}

// Initialize reset button
void initButton(){
    buttonReset.limitRate(100);
    buttonReset.setDebounceTime(200);
    buttonReset.setActivationHandler(resetSystem);
}

void setup(){
    initButton();
}

void loop(){
    buttonReset.poll();
    interface.receive();

    if(interfaceTick > PERIOD_INTERFACE){
        interface.transmit(); // Transmit temperature, current and status LEDs
        interfaceTick = 0;
    }

    if(controllerTick > PERIOD_CONTROLLER){
        controller.update(); // Read values and update PID control loop
        controllerTick = 0;
    }
}