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

        inline void ack(MSG msgToAck){comm.addFlagToken(MSG::ACK);comm.addFlagToken(msgToAck);}
        inline void nack(MSG msgToNAck){comm.addFlagToken(MSG::NACK);comm.addFlagToken(msgToNAck);}

    public:
        Interface(TemperatureController* controller);

        bool isConnected();

        // Receives and processes messages, sends updates back to host
        void update();

};

#endif