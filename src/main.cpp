/*
* 3D Bioprinter pneumatic
* 
*/
#include <Arduino.h>
#include <PID_v1.h> // PID library by Brett Beauregard
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <EEPROM.h>
#include "fastPWM.h"

// comment the following line for production code
//define debugging
// Minimum and maximum pressure of the sensor, in bar (or whatever other unit)
#define minPressure 0
#define maxPressure 4

// If the pressure goes above the maximum of the sensor, there is no way to control it.
// Keeping the max and min setpoints somewhat within the sensor's max and min
// helps prevent this issue.

#define minPressureSetpoint 0.98*minPressure
#define maxPressureSetpoint 0.98 *maxPressure

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 110;
uint8_t valveMaxValue = 200;

// Timers
//

/// The time between reads of the sensor, in milliseconds
unsigned long sensorReadPeriod_us = 200; //1
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;


/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms =1; //3
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

/// The time between reads of the analog pressure setpoint
unsigned long analogSetpointUpdatePeriod_ms = 100; //150
/// The last time the analog setpoint was updated
unsigned long analogSetpointLastUpdateTime_ms;

/// The time between transmission of data over serial, in milliseconds
unsigned long serialTransmissionPeriod_ms = 50; //15
/// The last time the data was sent
unsigned long serialLastSendTime_ms;

unsigned long pumpUpdatePeriod_ms = 500;
/// The last time the pump was on
unsigned long pumpLastOnTime;



// PID-related variables
//

double setPoint, currentPressure, bufferPressure;
float kp = 0.4;
float ki = 0.3;
float kd = 0;
float pidMaxOutput = 1;
float pidMinOutput = -1;
double pidOutput;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);
Adafruit_SSD1306 display(-1);

int lastAnalogSetpoint;

#define sensorAnalogPin A0 //pressure sensor Honeywell
#define sensorAnalogPin2 A1 // pressure sensor buffer cylinder
#define analogSetpointPin A4
#define analogSetpointRamps A3 
#define buttons A5 // remote buttons
#define pumpPin 8 // Pump motor 
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
void screen(float _setpoint, float _currentPressure);
// oled 1306 display


void setup()
{
    Serial.begin(115200);
    //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Timers
    sensorLastReadTime_us = millis();
    serialLastSendTime_ms = millis();
    lastControlTime_ms = millis();
    analogSetpointLastUpdateTime_ms = millis();
    

    // Setup pins
    pinMode(sensorAnalogPin, INPUT);
    pinMode(sensorAnalogPin2, INPUT);
    //pinMode(analogSetpointPin, INPUT);
    //pinMode(analogSetpointRamps, INPUT);
    //pinMode(rampsSol1, INPUT);
    //pinMode(rampsSol2, INPUT);
    //pinMode(rampsSol3, INPUT);
    //pinMode(buttons, INPUT);
    pinMode(pumpPin, OUTPUT);
    //pinMode(sol1, OUTPUT);
    //pinMode(sol2, OUTPUT);
    //pinMode(sol3, OUTPUT); 
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
    if (millis() - sensorLastReadTime_us > sensorReadPeriod_us) {
        readPressure();
        readPressure2();        
        sensorLastReadTime_us = millis();
    }

    if (millis() - serialLastSendTime_ms > serialTransmissionPeriod_ms) {
        sendSerialData();
        //screen(setPoint, currentPressure);
        serialLastSendTime_ms = millis();
    }

    if (millis() - pumpLastOnTime > pumpUpdatePeriod_ms) {
        pumpControl();        
        pumpUpdatePeriod_ms = millis();
    }

    
    // This loop takes ~250-300ms
    if (millis() - lastControlTime_ms > controlLoopPeriod_ms) {
        updateController(kp, ki, kd, setPoint);
        pid.Compute();
        handleControllerOutput();     
        lastControlTime_ms = millis();
       


    }

    if (millis() - analogSetpointLastUpdateTime_ms > analogSetpointUpdatePeriod_ms) {
        readAnalogSetpoint();        
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
        Serial.println(analogRead(sensorAnalogPin2));
        Serial.print(setPoint);
        Serial.print(", pressure: ");
        Serial.print(currentPressure);
        Serial.print(", pressure2:");
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
    currentPressure = abs(minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max));
    // Bound output between minimum and maximum pressure
    //currentPressure = max(minPressure, min(currentPressure, maxPressure));
}
void readPressure2() 
{
    float val = analogRead(sensorAnalogPin2);
    float max = 1023.; // max possible value of analogRead

    // transfer function for Honeywell HSC sensors is from 10% to 90% of possible values.
    bufferPressure = abs(minPressure + (maxPressure - minPressure) * (val - 0.11*max)/(0.8*max));
    // Bound output between minimum and maximum pressure
    //currentPressure = max(minPressure, min(currentPressure, maxPressure));
}
void handleControllerOutput()
{
    // Positive output: inlet valve is opened; vent is closed
    //uint8_t closedValue = 100;
    if (pidOutput >= 0) {
        setValve2(valveMinValue); //valveMinValue
        int val = valveMinValue + round(pidOutput * (valveMaxValue - valveMinValue));
        setValve1(val);
    }
    // Negative output: vent valve is opened; inlet is closed
    else {
        setValve1(valveMinValue);//valveMinValue
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

void screen(float _setpoint, float _currentPressure){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(F("now: "));
  display.setCursor(68, 0);
  display.print(_currentPressure, 1);
  display.setCursor(0, 17);
  display.print(F("set: "));
  display.setCursor(68, 17);
  display.print(_setpoint, 1);
  display.display();
}

void pumpControl()
{
    if (bufferPressure < 2){  
        digitalWrite(pumpPin, LOW);
    }
    else if (bufferPressure >= 3) 
    {
        digitalWrite(pumpPin, HIGH);
    }
}
