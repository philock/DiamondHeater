// Message types for serial communication
enum MSG : char{
    RESET, // Reset Microcontroller

    // Temperature and Current values
    T_SETPOINT,
    T_ACTUAL,
    CURRENT,

    // PID gain coefficients
    PID_P,
    PID_I,
    PID_D,

    STATUS,

    MSG_END // End of transmission
};
