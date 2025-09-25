#pragma once
class Sensor{
public:
    virtual ~Sensor() = default; // ~ means destructor, cleans up resources the object was using when the object is destroyed
    virtual double getValue() const = 0;
    virtual void reset() = 0;

    //for later
    void logValue() const;
};