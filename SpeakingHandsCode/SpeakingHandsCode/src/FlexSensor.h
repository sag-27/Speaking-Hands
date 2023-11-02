#include <Arduino.h>

class FlexSensor
{
private:
    int sensorPin;
    int minInput;
    int maxInput;
    int minOutput;
    int maxOutput;
    int numReadings;
    int readIndex;
    int *arryVals;

public:
    FlexSensor(int SensorPin, int NumReadings);
    void updateArryVals();
    void Calibrate();
    int getSensorValue(int raw);
    void setMinMaxInput(int MinInput, int MaxInput);
    int getMinInput();
    int getMaxInput();
};

// constructor in cpp file
FlexSensor::FlexSensor(int SensorPin, int NumReadings)
{
    this->sensorPin = SensorPin;
    this->minInput = 400;
    this->maxInput = 700;
    this->minOutput = 0;
    this->maxOutput = 1023;
    this->numReadings = NumReadings;
    arryVals = new int[numReadings];
    readIndex = 0;
    for (int i = 0; i < numReadings; i++)
        arryVals[i] = 0;
}

// updateArryVals to update only one element of an array instantaneouly (optional)
void FlexSensor::updateArryVals()
{
    readIndex++;
    if (readIndex >= numReadings)
        readIndex = 0;
    arryVals[readIndex] = analogRead(sensorPin);
}

// Calibrate in setup to calibrate flex sensor (optional)
void FlexSensor::Calibrate()
{
    for (int i = 0; i < 1000; i++)
    {
        if (i < 500)
        {
            updateArryVals();
        }
        else
        {
            int val = getSensorValue(1);
            if (val > maxInput)
                maxInput = val;
            if (val < minInput)
                minInput = val;
            delay(10);
        }
    }
}

// getSensorValue in loop to get running average smooth value of flex sensor
// each time getSensorValue is called, it will update one element of array and then calculate the running average smooth value
int FlexSensor::getSensorValue(int raw = 0)
{
    double runAvgSmooth = 0;
    updateArryVals();
    for (int i = 0; i < numReadings; i++)
    {
        runAvgSmooth += arryVals[i];
    }
    if (raw == 0)
    {
        int avgVal = (int)(runAvgSmooth / (double)numReadings);
        int map_avgVal = (avgVal - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
        return minOutput <= maxOutput ? constrain(map_avgVal, minOutput, maxOutput) : constrain(map_avgVal, maxOutput, minOutput);
    }
    else
    {
        return (int)(runAvgSmooth / (double)numReadings);
    }
}

// in setup for setting min and max values for the readings of flex sensor (optional)
void FlexSensor::setMinMaxInput(int MinInput, int MaxInput)
{
    this->minInput = MinInput;
    this->maxInput = MaxInput;
}

// in loop to returns minimum value of input read of the flexsensor (optional)
int FlexSensor::getMinInput()
{
    return minInput;
}

// in loop to returns maximum value of input read of the flexsensor (optional)
int FlexSensor::getMaxInput()
{
    return maxInput;
}
