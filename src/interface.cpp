#include <interface.h>

Interface::Interface(TemperatureController* controller_p) :
    comm(115200){
    controller = controller_p;
    indicator = &(controller_p->indicator);
    Serial.clear();
}

bool Interface::isConnected(){
    if (!bitRead(USB1_PORTSC1,7)) return true;
    else return false;
}

void Interface::update(){
    if(!isConnected()){
        controller->stop();
        return;
    }

    rx_message rxm = comm.getNextMsg();
    float payload;

    // Read messages until end of transmission
    while(rxm.msg != MSG::MSG_END){
        switch (rxm.msg){
            case MSG::PID_P:
                comm.getPayload(payload);
                controller->setKp(payload);
                break;
            case MSG::PID_I:
                comm.getPayload(payload);
                controller->setKi(payload);
                break;
            case MSG::PID_D:
                comm.getPayload(payload);
                controller->setKd(payload);
                break;
            case MSG::T_SETPOINT:
                comm.getPayload(payload);
                (controller->setpoint(payload))? ack(MSG::T_SETPOINT) : nack(MSG::T_SETPOINT);
                break;
            case MSG::START:
                (controller->start())? ack(MSG::START) : nack(MSG::START);
                break;
            case MSG::STOP:
                (controller->stop())? ack(MSG::STOP) : nack(MSG::STOP);
                break;
            case MSG::RESET:
                controller->reset();
                break;

            default:
                break;
        }

        rxm = comm.getNextMsg();
    }
    // Send acknowledgement flags
    comm.transmit();

    // Transmit state of fault, OT, OC and active LEDs back to host
    int state = indicator->getStatusWord();
    comm.addVariableToken(state, MSG::STATUS);

    // Transmit actual temperature
    float t = controller->getTemperature();
    comm.addVariableToken(t, MSG::T_ACTUAL);

    // Transmit actual temperature
    float I = controller->getCurrent();
    comm.addVariableToken(I, MSG::CURRENT);

    comm.transmit();
}