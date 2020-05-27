#include <Arduino.h>
#include <PID_v1.h> // PID library by Brett Beauregard
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "fastPWM.h"

// Minimum and maximum pressure of the sensor, in bar (or whatever other unit)
#define minPressure 0
#define maxPressure 10


double callibration = -0.025;
#define minPressureSetpoint 0.98*minPressure
#define maxPressureSetpoint 0.98 *maxPressure

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 120;
uint8_t valveMaxValue = 240;

// Timers
//

/// The time between reads of the sensor, in microseconds
unsigned long sensorReadPeriod_us = 200; //1, 200
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;

/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms = 1; //3 , 1
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

/// The time between reads of the analog pressure setpoint
unsigned long analogSetpointUpdatePeriod_ms = 100; //150, 100
/// The last time the analog setpoint was updated
unsigned long analogSetpointLastUpdateTime_ms;

/// The time between transmission of data over I2C, in milliseconds
unsigned long displayTransmissionPeriod_ms = 50; //15, 50
/// The last time the data was sent
unsigned long displayLastSendTime_ms;

unsigned long pumpUpdatePeriod_ms = 200;
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
uint8_t valve1Value = 0;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);
Adafruit_SSD1306 display(-1);

int lastAnalogSetpoint;

#define sensorAnalogPin A0 //pressure sensor Honeywell
#define sensorAnalogPin2 A1 // pressure sensor buffer cylinder
#define analogSetpointPin A3 //Setpoint
#define analogSetpointPinRamps A2
#define pumpPin 8 // Pump motor 
#define PWM_PIN 4 //ramps setpoint 

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
/// Open and close valves based on PID controller output
void handleControllerOutput();
/// Read analog SP pin and update the pressure setpoint
void readAnalogSetpoint();
// oled 1306 display
void screen(float _setpoint, float _currentPressure, float _sp, float _pid);
void serialData();

float rawValue = 0;
//#define debugging
void setup()
{
    Serial.begin(115200);

    #ifdef debugging
    // Wait for serial to attach, but only for debugging purposes
        while(!Serial);
    #endif

    #ifndef debugging
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    #endif

    // Timers
    sensorLastReadTime_us = micros();
    displayLastSendTime_ms = millis();
    lastControlTime_ms = millis();
    analogSetpointLastUpdateTime_ms = millis();
    pumpLastOnTime = millis();
    

    // Setup pins
    pinMode(sensorAnalogPin, INPUT);
    pinMode(sensorAnalogPin2, INPUT);
    pinMode(pumpPin, OUTPUT);
    pwm613configure(PWM47k);
    pwm91011configure(PWM8k);
    pinMode(PWM_PIN, INPUT);

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
        sensorLastReadTime_us = micros();
    }

    if (millis() - displayLastSendTime_ms > displayTransmissionPeriod_ms) {
        screen(currentPressure, bufferPressure, setPoint, pidOutput); 
        //serialData();
        displayLastSendTime_ms = millis();
    }



    if (millis() - pumpLastOnTime > pumpUpdatePeriod_ms) {
        pumpControl();        
        pumpLastOnTime = millis();
    }

    
    if (millis() - lastControlTime_ms > controlLoopPeriod_ms) {
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
    //analogWrite(13,val);
    pwmSet13(val);
}

void setValve2(uint8_t val)
{
    pwmSet6(val);
}

void readPressure()
{
    float val = analogRead(sensorAnalogPin);
    float max = 1023.; // max possible value of analogRead    
    currentPressure = abs(minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max));
    currentPressure += callibration;
}
void readPressure2() 
{
    float fullScale = 9630.; // max possible value of analogRead
    float offset = 360;
    rawValue = 0; //analogRead(sensorAnalogPin2);
    for (int x = 0; x < 10; x++) rawValue = rawValue + analogRead(sensorAnalogPin2);
    bufferPressure = (rawValue - offset) * 7.0 / (fullScale - offset); // pressure conversion
    
}
void handleControllerOutput()
{
    // Positive output: inlet valve is opened; vent is closed
    //uint8_t closedValue = 100;
    if (pidOutput >= 0) {
        setValve2(valveMinValue); //valveMinValue
        int val = valveMinValue + round(pidOutput * (valveMaxValue - valveMinValue));
        valve1Value = val;
        setValve1(val);
    }
    // Negative output: vent valve is opened; inlet is closed
    else {
        setValve1(valveMinValue);//valveMinValue
        int val = valveMinValue + round(-pidOutput * (valveMaxValue - valveMinValue));
        setValve2(val);
    }

    if (setPoint == 0) {
        setValve1(0);        
    }
}

void readAnalogSetpoint()
{
    /* int val = analogRead(analogSetpointPinRamps);
    val += analogRead(analogSetpointPinRamps);
    val += analogRead(analogSetpointPinRamps);
    val += analogRead(analogSetpointPinRamps);
    val += analogRead(analogSetpointPinRamps);
    val /= 5.;

    if (abs(val - lastAnalogSetpoint) >= 10) {
        lastAnalogSetpoint = val;
        float s = float(val) * (4) / 1023.;
        setPoint = s;
    }
    */
   int pwm_value = pulseIn(PWM_PIN, HIGH);
   float s = float(pwm_value) * 4/ 2019.;
   setPoint = s;
}



void serialData()
{
    if (Serial) {
        Serial.print(rawValue);
        Serial.print(setPoint);
        Serial.print(",");
        Serial.print(bufferPressure);
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
        
    }
}

/*void screen(float _setpoint, float _currentPressure){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(F("now: "));
  display.setCursor(68, 0);
  display.print(_currentPressure, 2);
  display.setCursor(0, 17);
  display.print(F("set: "));
  display.setCursor(68, 17);
  display.print(_setpoint, 2);
  display.display();
}
*/
void screen(float _hp, float _bp, float _sp, float _pid){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print(F("Head pressure"));
    display.setCursor(98,0);
    display.print(_hp,2);
    display.setCursor(0,8);
    display.print(F("Buffer pressure "));
    display.setCursor(98,8);
    display.print(_bp,2);
    display.setCursor(0,16);
    display.print("Set point");
    display.setCursor(98,16);
    display.print(_sp, 2);
    display.setCursor(0,24);
    display.print("PID output");
    display.setCursor(98,24);
    display.print(_pid ,2);

    display.display();


}
void pumpControl()
{
    if (bufferPressure < 3.1){  
        digitalWrite(pumpPin, LOW);
    }
    else if (bufferPressure >= 3.2) 
    {
        digitalWrite(pumpPin, HIGH);
    }
}
