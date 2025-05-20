#ifndef INTERFACE_H
#define INTERFACE_H
#include <comm.h>
#include <temperatureController.h>
#include <statusIndicator.h>

class Interface{
    private:
        Comm comm;
        TemperatureController* controller;
        StatusIndicator* indicator;

        inline void ack(){comm.addFlagToken(MSG::ACK);};
        inline void nack(){comm.addFlagToken(MSG::NACK);};

    public:
        Interface(TemperatureController* controller, StatusIndicator* indicator);

        // Receives and processes messages, sends updates back to host
        void update();

};

#endif