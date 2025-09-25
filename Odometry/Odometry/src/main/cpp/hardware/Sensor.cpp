#include "hardware/Sensor.h"
#include <iostream>

void Sensor::logValue() const{
    std::cout << "Sensor value: " << getValue() << std::endl;
}