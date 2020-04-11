/*
* 3D Bioprinter pneumatic
* Author: Sasho Sotirov
* *
*/

#include <Arduino.h>
#include <PID_v1.h> // PID library by Brett Beauregard
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <EEPROM.h>
#include "fastPWM.h"

// comment the following line for production code
//define debugging
// Minimum and maximum pressure of the sensor, in bar (or whatever other unit)
#define minPressure 0
#define maxPressure 10.0

// If the pressure goes above the maximum of the sensor, there is no way to control it.
// Keeping the max and min setpoints somewhat within the sensor's max and min
// helps prevent this issue.

#define minPressureSetpoint 0.98*minPressure
#define maxPressureSetpoint 0.98 *maxPressure

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 170;
uint8_t valveMaxValue = 255;

// Timers
//

/// The time between reads of the sensor, in microseconds
unsigned long sensorReadPeriod_us = 200;
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;


/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms = 1;
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

/// The time between reads of the analog pressure setpoint
unsigned long analogSetpointUpdatePeriod_ms = 100;
/// The last time the analog setpoint was updated
unsigned long analogSetpointLastUpdateTime_ms;

/// The time between transmission of data over serial, in milliseconds
unsigned long serialTransmissionPeriod_ms = 50;
/// The last time the data was sent
unsigned long serialLastSendTime_ms;



// PID-related variables
//

double setPoint, currentPressure, bufferPressure;
float kp = 0.6;
float ki = 0.3;
float kd = 0;
float pidMaxOutput = 1;
float pidMinOutput = -1;
double pidOutput;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);
Adafruit_SSD1306 display;

int lastAnalogSetpoint;

#define sensorAnalogPin A0 //pressure sensor Honeywell
#define sensorAnalogPin2 A1 // pressure sensor buffer cylinder
#define analogSetpointPin A4
#define analogSetpointRamps A3 
#define buttons A5 // remote buttons
#define pumpPin 11 // Pump motor 
#define rampsSol1 4
#define rampsSol2 3
#define rampsSol3 2
#define sol1 8
#define sol2 9
#define sol3 10 

 
// Function headers
//
/// Set valve openings. 0: fully closed; 255: fully open
void setValve1(uint8_t val);
void setValve2(uint8_t val);
/// Send all the useful values (setpoint, pv, etc.. over USB)
void sendSerialData();
/// Read the current pressure and update the currentPressure variable
void readPressure();
void readPressure2();
void pumpControl();
/// Update the PID constants and/or setpoint
void updateController(float kp_, float ki_, float kd_, float setpoint_);
/// Update the setpoint only
void updateController(float setpoint_);
/// Open and close valves based on PID controller output
void handleControllerOutput();
/// Read analog SP pin and update the pressure setpoint
void readAnalogSetpoint();
void screen(float _setpoint);
// oled 1306 display


void setup()
{
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

#ifdef debugging
    // Wait for serial to attach, but only for debugging purposes
    while(!Serial);
#endif

    // Timers
    sensorLastReadTime_us = micros();
    serialLastSendTime_ms = millis();
    lastControlTime_ms = millis();
    analogSetpointLastUpdateTime_ms = millis();
    

    // Setup pins
    pinMode(sensorAnalogPin, INPUT);
    pinMode(sensorAnalogPin2, INPUT);
    pinMode(analogSetpointPin, INPUT);
    pinMode(analogSetpointRamps, INPUT);
    pinMode(rampsSol1, INPUT);
    pinMode(rampsSol2, INPUT);
    pinMode(rampsSol3, INPUT);
    pinMode(buttons, INPUT);
    pinMode(pumpPin, OUTPUT);
    pinMode(sol1, OUTPUT);
    pinMode(sol2, OUTPUT);
    pinMode(sol3, OUTPUT);    
    pwm613configure(PWM47k);
    pwm91011configure(PWM8k);

    setValve1(0);
    setValve2(0);
    
    lastAnalogSetpoint = 0;
    setPoint = 0;
   
    pid.SetOutputLimits(pidMinOutput, pidMaxOutput);
    pid.SetSampleTime(controlLoopPeriod_ms);
    pid.SetMode(AUTOMATIC);
    pid.SetTunings(kp, ki, kd);
}


void loop()
{
    if (micros() - sensorLastReadTime_us > sensorReadPeriod_us) {
        readPressure();
        readPressure2();
        sensorLastReadTime_us = micros();
    }

    if (millis() - serialLastSendTime_ms > serialTransmissionPeriod_ms) {
        sendSerialData();
        serialLastSendTime_ms = millis();
    }

    
    // This loop takes ~250-300ms
    if (millis() - lastControlTime_ms > controlLoopPeriod_ms) {
        updateController(kp, ki, kd, setPoint);
        pid.Compute();
        handleControllerOutput();        
        pumpControl();
        lastControlTime_ms = millis();
    }

    if (millis() - analogSetpointLastUpdateTime_ms > analogSetpointUpdatePeriod_ms) {
        readAnalogSetpoint();
        //screen(setPoint,currentPressure);
        analogSetpointLastUpdateTime_ms = millis();
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
        Serial.print(", pressure");
        Serial.print(currentPressure);
        Serial.print(", pressure2: ");
        Serial.print(bufferPressure);
        Serial.print(",");
        Serial.print(kp);
        Serial.print(",");
        Serial.print(ki);
        Serial.print(",");
        Serial.print(kd);
        Serial.print(", pidOUT:");
        Serial.println(pidOutput);
    }
}

void updateController(float kp_, float ki_, float kd_, float setpoint_)
{
    if (setpoint_ != setPoint && setpoint_ >= minPressureSetpoint && setpoint_ <= maxPressureSetpoint) {
        setPoint = setpoint_;

        // If the setpoint is negative (and we are pulling a vacuum), then opening the
        // input will lower the pressure, and opening the vent will increase it. This is the opposite
        // of what happens with positive pressure, so the controller direction should be switched accordingly.
        if (setPoint < 0 && pid.GetDirection() == DIRECT)
            pid.SetControllerDirection(REVERSE);
        else if (setPoint > 0 && pid.GetDirection() == REVERSE)
            pid.SetControllerDirection(DIRECT);
    }

    if (kp_ != kp || ki_ != ki || kd_ != kd) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        pid.SetTunings(kp, ki, kd);
        
    }
}

void updateController(float setpoint_)
{
    updateController(kp, ki, kd, setpoint_);
}

void readPressure()
{
    float val = analogRead(sensorAnalogPin);
    float max = 1023.; // max possible value of analogRead

    // transfer function for Honeywell HSC sensors is from 10% to 90% of possible values.
    currentPressure = minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max);
    // Bound output between minimum and maximum pressure
    //currentPressure = max(minPressure, min(currentPressure, maxPressure));
}
void readPressure2() // MPX5700 pressure sensor (700kPa)
{
  float rawValue = 0; // A/D readings
  int fullScale = 9630;
  int offset = 346; // zero pressure adjust
  //int fullScale = 9630; // max pressure (span) adjust
  for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(sensorAnalogPin2);
  bufferPressure = (rawValue - offset) * 7.0 / (fullScale - offset); // pressure conversion;// pressure conversion in bar
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
}

void readAnalogSetpoint()
{
    int val = analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val /= 5.;

    // to avoid changing the setpoint 100 times a second, it is only updated if it has changed substantially
    if (abs(val - lastAnalogSetpoint) >= 10) {
        lastAnalogSetpoint = val;
        float s = minPressureSetpoint + float(val) * (maxPressureSetpoint - minPressureSetpoint) / 1023.;
        setPoint = s;
    }
}

void screen(float _setpoint) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextColor(INVERSE);
  display.setCursor(0, 0);
  display.print(F("now: "));
  display.setCursor(68, 0);
  display.print(_setpoint, 2);
  display.setCursor(0, 17);
  display.print(F("set: "));
  display.setCursor(68, 17);
  display.print(_setpoint, 2);
  display.display();
}

void pumpControl()
{
    if (bufferPressure <= setPoint){
        digitalWrite(pumpPin, HIGH);
    }
    else{
        digitalWrite(pumpPin, LOW);
    }
}
