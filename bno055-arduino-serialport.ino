// by Francisco Bernardo: February 2018 
// Includes public domain code adapted from Kevin Townsend https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
// Includes public domain code adapted from Rebecca Fiebrink: October 2017
// Code originally from http://www.instructables.com/id/Arduino-to-Processing-Serial-Communication-withou/
// by https://www.instructables.com/member/thelostspore/
// Code was shared under public domain https://creativecommons.org/licenses/publicdomain/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// This code reads analog inputs from pins A0 and A1 and sends these values out via serial
// You can add or remove pins to read from, but be sure they are separated by commas, and print a
// newline character at the end of each loop()

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int AnalogPin0 = A0; //Declare an integer variable, hooked up to analog pin 0
int AnalogPin1 = A1; //Declare an integer variable, hooked up to analog pin 1
int AnalogPin2 = A2; //Declare an integer variable, hooked up to analog pin 2
void setup(void) 
{
  Serial.begin(9600); //Begin Serial Communication with a baud rate of 9600  
  setupBNO055();
}


void loop(void) 
{
  int AnalogPin0Value = analogRead(AnalogPin0);
  int AnalogPin1Value = analogRead(AnalogPin1);
//  int AnalogPin2Value = analogRead(AnalogPin2);

  // Light resistor
//  Serial.print(AnalogPin0Value, DEC); 
//  Serial.print(",");

  
//  Serial.print(AnalogPin1Value, DEC); 
//  Serial.print(",");
//  Serial.print(AnalogPin2Value, DEC); 
//  Serial.print(",");

  // IMU sensor
  send_IMU_Quaternion(false);
//  Serial.print(",");
//  send_Euler_degrees();  
//  Serial.print(",");
//  send_Accelerometer();
//  Serial.print(",");
//  send_Magnetometer();
//  Serial.print(",");
//  send_Gravity();
//  Serial.print(",");
//  send_Current_Temperature();

  Serial.println();
  
  //delay(100);
}

void setupBNO055()
{
//  Serial.println("Orientation Sensor Test"); 
//  Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);  

  // Possible vector values can be:
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees 
  // - VECTOR_LINEARACCEL   - m/s^2
}


void send_Euler_degrees()
{
  /* Display the Euler degrees */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
//  Serial.print("X: ");
  Serial.print(euler.x(), DEC);
  Serial.print(",");
//  Serial.print(" Y: ");
  Serial.print(euler.y(), DEC);
  Serial.print(",");
//  Serial.print(" Z: ");
  Serial.print(euler.z(), DEC);
}

void send_IMU_Quaternion(bool printLabels)
{
    /* Display the quat data */
  imu::Quaternion quat = bno.getQuat();
  
  if(printLabels) Serial.print("qW: ");
  Serial.print(quat.w(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print(" qX: ");
  Serial.print(quat.y(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print(" qY: ");
  Serial.print(quat.x(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print(" qZ: ");
  Serial.print(quat.z(), DEC);
//  Serial.print(",");
//  Serial.println("");
}

void send_Accelerometer(bool printLabels)
{
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  /* Display the floating point data - m/s^2 */
  if(printLabels) Serial.print("X: ");
  Serial.print(accel.x(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print(" Y: ");
  Serial.print(accel.y(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print(" Z: ");
  Serial.print(accel.z(), DEC);
}

void send_Magnetometer(bool printLabels)
{
  /* Display the Euler degrees */
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data - uT */
  if(printLabels) Serial.print("magnetometer_X: ");
  Serial.print(magnetometer.x(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print("magnetometer_Y: ");
  Serial.print(magnetometer.y(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print("magnetometer_Z: ");
  Serial.print(magnetometer.z(), DEC);
}

void send_Current_Temperature(bool printLabels)
{
  /* Display the current temperature - C */
  int8_t temp = bno.getTemp();
  
  if(printLabels) Serial.print("Current Temperature: ");
  Serial.print(temp);
//  Serial.print(" C");
}

void send_Gravity(bool printLabels)
{
  /* Display the gravity -  m/s^2 */
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  /* Display the floating point data */
  if(printLabels) Serial.print("Gravity_X (m/s^2): ");
  Serial.print(gravity.x(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print("Gravity_Y: ");
  Serial.print(gravity.y(), DEC);
  Serial.print(",");
  if(printLabels) Serial.print("Gravity_Y: ");
  Serial.print(gravity.z(), DEC);
}
