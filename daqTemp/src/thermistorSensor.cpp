/* thermistorSensor.cpp */

#include <Arduino.h>
#include "thermistorSensor.h"

// Implementation of the thermistorSensor class

//
// Reads the value of the temperature sensor, and returns in Celsius
//
float ThermistorSensor::Read()
{
    // Find the resistance of the thermistor
    float rawADC = analogRead(this->_sensorPin);
    // Serial.printf("Raw Adc: %d\n", rawADC);
    float vout = (rawADC * this->_vcc) / this->_vccRes;
    // Serial.printf("Vout: %0.2f\n", vout);
    float rth = ((this->_vcc * this->_r2) / vout) - this->_r2;
    // Serial.printf("Rth: %0.2f\n", rth);
    // Calculate the temperature
    float temperature = (1 / (this->_a + (this->_b * log(rth) + (this->_c * pow(log(rth), 3)))));
    // Serial.println(temperature);
    // Convert from K to C
    return temperature - 273.15;
}

//
// Prints the temperature read to the serial montitor
//
void ThermistorSensor::Print()
{
    float temperature = this->Read();
    Serial.printf("Reading %0.2f \u00B0C\n", temperature);
}

//
// Set voltage settings
//
void ThermistorSensor::SetHardwareConfig(float vcc, float vccRes)
{
    this->_vcc = vcc;
    this->_vccRes = vccRes;
}

//
// Sets the Steinhart-Hart Coefficents.
// To maintain the previous value, pass in -1
//
void ThermistorSensor::SetSteinhartHartCoefficents(float a, float b, float c)
{
    if (a != -1) this->_a = a;
    if (b != -1) this->_b = b;
    if (c != -1) this->_c = c;
}