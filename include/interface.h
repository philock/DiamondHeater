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

        elapsedMillis _comm_watchdog;

        //bool teststate1;
        //bool teststate2;

    public:
        Interface(TemperatureController* controller);

        bool isConnected();

        // Receives and processes messages, only runs when data available
        void receive();

        // Transmit updates back to host, call at fixed interval
        void transmit();



};

#endif