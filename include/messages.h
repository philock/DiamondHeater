// Message types for serial communication
enum MSG : char{
    RESET, // Reset Microcontroller (flag)
    START, // Start system (temperature control). Returns Ack/Nack
    STOP,  // Stop system (temperature control). Returns Ack/Nack
    ACK,   // Acknowledge transmission (flag)
    NACK,  // Not acknowledge transmission (flag)

    // Temperature and Current values (float)
    T_SETPOINT, // Set new setpoint temperature. Returns Ack/Nack
    T_ACTUAL,   // Actual temperature, only sent back to host
    CURRENT,    // Actual current, only sent back to host

    // Set PID gain coefficients (float)
    PID_P,
    PID_I,
    PID_D,

    // Indicator LEDs status word (int)
    STATUS,

    MSG_END // End of transmission
};
