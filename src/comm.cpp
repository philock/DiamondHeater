#include <comm.h>

Comm::Comm(int baud){
    baudrate = baud;
    Serial.begin(baudrate);
    Serial.setTimeout(100);
    Serial.clear();
    
    txBufPos = 0;
}

void Comm::transmit(){
    if(txBufPos > 0){
        // Terminate message
        txBuf[txBufPos] = MSG::MSG_END;

        // Send over Serial
        Serial.write(txBuf, txBufPos + 1);

        // Clear transmit buffer
        memset(txBuf, 0, BUF_SIZE); 
        txBufPos = 0;
    }
}

rx_message Comm::getNextMsg(){
    if (!Serial.available()){
        rxm.msg = MSG::MSG_END;
        rxm.msg_type = MSG_TYPE::MSG_FLAG;
        rxm.size = 0;
        return rxm;
    }
    char prefix;
    Serial.readBytes(&prefix, 1);

    char type = ((prefix >> 6) & 0b11);
    char msg = prefix & 0b00111111;

    rxm.msg      = (MSG)msg;
    rxm.msg_type = (MSG_TYPE)type;

    switch(type){
        case MSG_TYPE::MSG_FLAG:
            rxm.size = 0;
            break;

        case MSG_TYPE::MSG_VARIABLE:
            rxm.size = -1;
            break;

        case MSG_TYPE::MSG_CUSTOM:
            char size;
            Serial.readBytes(&size, 1);
            rxm.size = (int)size;
            break;
    }

    return rxm;
}

bool Comm::addFlagToken(MSG identifier){
    char prefix = (MSG_TYPE::MSG_FLAG << 6) | identifier;
    return appendToken(&prefix, 1);
}

bool Comm::appendToken(const char* token, int length){
    // check that token fits into tx buffer, while leaving space for END Token
    if (txBufPos + length >= BUF_SIZE ){
        return false;
    }

    // append token at end of tx buffer
    memcpy(&txBuf[txBufPos], token, length);

    txBufPos += length;

    return true;
}