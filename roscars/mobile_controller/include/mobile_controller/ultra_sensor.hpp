#ifndef ULTRA_SENSOR_HPP
#define ULTRA_SENSOR_HPP

#include <gpiod.h>
#include <chrono>

class UltraSensor {
public:
    UltraSensor();
    ~UltraSensor();
    float read_distance();

private:
    struct gpiod_chip *chip;
    struct gpiod_line *trig_line;
    struct gpiod_line *echo_line;
};

#endif  // ULTRA_SENSOR_HPP
