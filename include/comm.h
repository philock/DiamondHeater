#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
#include <messages.h>

#define BUF_SIZE 128

enum MSG_TYPE : char{
    MSG_FLAG     = 0x00, 
    MSG_VARIABLE = 0x01, 
    MSG_CUSTOM   = 0x02  
};

struct rx_message {
    MSG msg;
    MSG_TYPE msg_type;
    int size;
};

class Comm{
    public:
        Comm(int baud);

        bool dataAvailable();

        void transmit();

        template<typename T>
        bool addVariableToken(const T& object, MSG identifier){
            int size = sizeof(T);
            if(size > 255) return false;

            // Prepare token
            int toklength = size + 1;
            char token[toklength];

            // Add Prefix
            MSG_TYPE msgtyp = MSG_TYPE::MSG_VARIABLE;
            char prefix = (msgtyp << 6) | identifier;
            token[0] = prefix;

            // Add Data
            memcpy(&token[1], &object, size);

            return appendToken(token, toklength);
        }

        template<typename T>
        bool addCustomToken(const T& object, MSG identifier, int size){
            if(size > 255) return false;

            // Prepare token
            int toklength = size + 2;
            char token[toklength];

            // Add Prefix
            MSG_TYPE msgtyp = MSG_TYPE::MSG_CUSTOM;
            char prefix = (msgtyp << 6) | identifier;
            token[0] = prefix;
            token[1] = size;

            // Add Data
            memcpy(&token[2], &object, size);
            
            return appendToken(token, toklength);
        }

        bool addFlagToken(MSG identifier);

        // Call to get identifier and size of next message
        rx_message getNextMsg();

        // Call after getNextMsg() to read payload
        template<typename T>
        bool getPayload(T& object){
            if(rxm.msg_type == MSG_TYPE::MSG_FLAG) return false; // No Payload

            int size;
            if(rxm.msg_type == MSG_TYPE::MSG_VARIABLE) size = sizeof(T);
            else size = rxm.size; // Custom size message

            char data[size];
            Serial.readBytes(data, size);

            memcpy(&object, data, size);

            return true;
        }


    private:
        int baudrate;

        char txBuf[BUF_SIZE];
        int txBufPos;

        rx_message rxm;

        bool appendToken(const char* token, int length);

};

#endif