#include <Arduino.h>
#include <PID_v1.h> // PID library by Brett Beauregard
#include <Wire.h>
#include "fastPWM.h"

// comment the following line for production code
//#define debugging

// Minimum and maximum pressure of the sensor, in PSI (or whatever other unit)
#define minPressure 0
#define maxPressure 10
// Calibration: if the actual pressure output doesn't correspond to the given sensor value,
// set this value to be the observed difference.
// e.g. if the regulator outputs 10 PSI when it is actually at 12 PSI, then set +2.0.

// Note: this has only been observed with 5V analog sensors, and is probably due to errors
// introduced by the conversion to 3.3V. The sensors themselves are calibrated and should be
// accurate unless there is manufacturing defect.
double calibration = 0;

// If the pressure goes above the maximum of the sensor, there is no way to control it.
// Keeping the max and min setpoints somewhat within the sensor's max and min
// helps prevent this issue.
// The setpoint and current presure transmitted via analog or i2c interfaces
// are both bounded by these values. E.g. if using a 30psi sensor, a value
// of 3.3v on analog or 255 on i2c corresponds to 29.5 psi, not 30.
#define minPressureSetpoint 0
#define maxPressureSetpoint 3.5
int i2cAddress = 43;

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 80;
uint8_t valveMaxValue = 255;

// Timers
//

/// The time between reads of the sensor, in microseconds
unsigned long sensorReadPeriod_us = 200;
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;

/// The time between transmission of data over serial, in milliseconds
unsigned long serialTransmissionPeriod_ms = 50;
/// The last time the data was sent
unsigned long serialLastSendTime_ms;

/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms = 1;
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

// PID-related variables
//

double setPoint, currentPressure;
float kp = 0.4;
float ki = 0.3;
float kd = 0;
float pidMaxOutput = 1;
float pidMinOutput = -1;
double pidOutput;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);
#define sensorAnalogPin A0 

// Function headers
//

/// Set valve openings. 0: fully closed; 255: fully open
void setValve1(uint8_t val);
void setValve2(uint8_t val);
/// Send all the useful values (setpoint, pv, etc.. over USB)
void sendSerialData();
/// Read incoming serial data and update the controller
void processSerialData();
/// Read the current pressure and update the currentPressure variable
void readPressure();
/// Update the setpoint only
void updateController(float setpoint_);
/// Open and close valves based on PID controller output
void handleControllerOutput();
/// Called when data is received via i2c
void i2cReceiveEvent(int nBytes);
/// Called when data is requested via i2c
void i2cRequestEvent();

void setup()
{
#ifdef debugging
    // Wait for serial to attach, but only for debugging purposes
    Serial.begin(115200);
    while(!Serial);
#endif

    // Timers
    sensorLastReadTime_us = micros();
    serialLastSendTime_ms = millis();
    lastControlTime_ms = millis();
    
    // Setup pins
    pinMode(sensorAnalogPin, INPUT);
    pwm613configure(PWM47k);
    pwm91011configure(PWM8k);

    // i2c communication
    Wire.begin(i2cAddress);
    Wire.onReceive(i2cReceiveEvent);
    Wire.onRequest(i2cRequestEvent);

    setValve1(0);
    setValve2(0);
    
    setPoint = 0;

    kp = 0.9; //0.4,0.6(!!) 
    ki = 0.3; //0.3
    kd = 0;

    pid.SetOutputLimits(pidMinOutput, pidMaxOutput);
    pid.SetSampleTime(controlLoopPeriod_ms);
    pid.SetMode(AUTOMATIC);
    pid.SetTunings(kp, ki, kd);
}


void loop()
{
    if (micros() - sensorLastReadTime_us > sensorReadPeriod_us) {
        readPressure();
        sensorLastReadTime_us = micros();
    }

    if (millis() - serialLastSendTime_ms > serialTransmissionPeriod_ms) {
        sendSerialData();
        serialLastSendTime_ms = millis();
    }

    // This loop takes ~250-300ms
    if (millis() - lastControlTime_ms > controlLoopPeriod_ms) {
        pid.Compute();
        lastControlTime_ms = millis();
        handleControllerOutput();
    }

}

void setValve1(uint8_t val)
{
    pwmSet13(val);
}

void setValve2(uint8_t val)
{
    pwmSet6(val);
}

void sendSerialData()
{
    if (Serial) {
        Serial.print(setPoint);
        Serial.print(",");
        Serial.print(currentPressure);
        Serial.print(",");
        Serial.print(kp);
        Serial.print(",");
        Serial.print(ki);
        Serial.print(",");
        Serial.print(kd);
        Serial.print(",");
        Serial.println(pidOutput);
        Serial.print(": ");
    }
}

void updateController(float setpoint_)
{
    if (setpoint_ != setPoint && setpoint_ >= minPressureSetpoint && setpoint_ <= maxPressureSetpoint) {
        setPoint = setpoint_;
    }
}

void readPressure()
{
    float val = analogRead(sensorAnalogPin);
    float max = 1023.; // max possible value of analogRead

    // transfer function for Honeywell HSC sensors is from 10% to 90% of possible values.
    currentPressure = minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max);
    // Bound output between minimum and maximum pressure
    //currentPressure = max(minPressure, min(currentPressure, maxPressure));
    currentPressure += calibration;

}

void handleControllerOutput()
{
    // Positive output: inlet valve is opened; vent is closed
    if (pidOutput >= 0) {
        setValve2(valveMinValue);
        int val = valveMinValue + round(pidOutput * (valveMaxValue - valveMinValue));
        setValve1(val);
    }
    // Negative output: vent valve is opened; inlet is closed
    else {
        setValve1(valveMinValue);
        int val = valveMinValue + round(-pidOutput * (valveMaxValue - valveMinValue));
        setValve2(val);
    }

    // Save a tiny bit of power at idle
    if (setPoint == 0) {
        setValve1(0);
    }
}


void i2cRequestEvent()
{
    // send the current pressure, between 0 and 255
    int val = round((currentPressure - minPressureSetpoint)/(maxPressureSetpoint - minPressureSetpoint) * 255.);
    uint8_t pv = max(0, min(val, 255));
    Wire.write(pv);
}

void i2cReceiveEvent(int nBytes)
{
    uint8_t sp_ = Wire.read();
    float sp_psi = minPressureSetpoint + float(sp_) * (maxPressureSetpoint - minPressureSetpoint) / 255.;
    updateController(sp_psi);
    //updateController(0.38);
}