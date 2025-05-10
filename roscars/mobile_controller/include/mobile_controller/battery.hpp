#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <cstdint>

class Battery {
public:
    Battery(int min_value = 2396, int max_value = 3215);
    ~Battery();
    
    float calculate_percentage(int value);
    float get_battery(int sample_count = 10);
    
private:
    int file;
    const char *bus = "/dev/i2c-1"; // I2C 버스 주소
    int min_value, max_value;
    
    int read_adc_value();  // private 함수로 읽기
};

#endif  // BATTERY_HPP
