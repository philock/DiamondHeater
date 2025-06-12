#ifndef CONFIG_H
#define CONFIG_H

// Circuit parameters
#define R_SENSE   0.025    // Current sense resistor (Ohms)
#define I_GAIN    26.45455 // Gain of the current feedback amplifier (1 + R1/R2)
#define R_REF     4000.0   // Value of the Rref resistor. 4300.0 for PT1000
#define R_NOMINAL 1000.0   // 'Nominal' 0 °C resistance of the RTD sensor
#define V_REF     3.812    // DAC reference voltage
#define OFFSET_COMP -0.01  // Added to current setpoint to compensate for offset
#define GAIN_COMP 1.016    // Units amps/amp. Compensates gain error of analog circuit
#define ADC_GAIN_COMP 0.980324245 // Units amps/amp

// System parameters
#define PERIOD_CONTROLLER 100  // Period at which system PID gets updated in milliseconds
#define PERIOD_INTERFACE  250  // Period at which communication with host system is handled in milliseconds
#define COMM_WATCHDOG     5000 // Host system acknowledges reception. If the controller does not receive this Ack over five seconds, it resets

// Maximum values
#define I_MAX 4.6 // Maximum output current (A)
#define I_OCP 4.8 // Current at which over-current protection activates (A)
#define T_MAX 200 // Maximum temperature (°C)

// Control loop default PID parameters
#define PIDKp 0.045
#define PIDKi 0.01
#define PIDKd 0.0

// Pin definitions
#define PIN_FAULT   4
#define PIN_OC      3
#define PIN_OT      2
#define PIN_ACTIVE  1
#define PIN_POWER   0
#define PIN_CS      10
#define PIN_SYNC    9
#define PIN_RESET   17
#define PIN_ADC     15

#endif