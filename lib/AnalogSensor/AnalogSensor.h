#ifndef _ANALOGSENSOR_H
#define _ANALOGSENSOR_H

#include "Arduino.h"

class AnalogSensor
{
private:
    uint8_t _pin;
    float (*_equation)(float, float[], uint8_t);
    float *coeff;
    uint8_t _numCoeff;

public:
    AnalogSensor(/* args */);
    ~AnalogSensor();
    void setup(const uint8_t, float (*equation)(float, float[], uint8_t), float coeff[], uint8_t numCoeff);
    float read();
};

AnalogSensor::AnalogSensor(/* args */)
{
}

AnalogSensor::~AnalogSensor()
{
}


#endif //_ANALOGSENSOR_H
