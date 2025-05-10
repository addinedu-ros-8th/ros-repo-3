#include "battery.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <vector>

Battery::Battery(int min_value, int max_value)
    : min_value(min_value), max_value(max_value) {

    if ((file = open(bus, O_RDWR)) < 0) {
        std::cerr << "I2C 버스 열기 실패\n";
        exit(1);
    }

    int addr = 0x48; // ADS1115 기본 주소
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        std::cerr << "ADS1115 디바이스 연결 실패\n";
        exit(1);
    }
}

Battery::~Battery() {
    close(file);
}

float Battery::calculate_percentage(int value) {
    if (value < min_value)
        return 0.0f;
    else if (value > max_value)
        return 100.0f;
    else
        return static_cast<float>(value - min_value) / (max_value - min_value) * 100.0f;
}

float Battery::get_battery(int sample_count) {
    std::vector<int> values;
    for (int i = 0; i < sample_count; i++) {
        values.push_back(read_adc_value());
        usleep(100000);  // 0.1초 대기
    }

    int sum = 0;
    for (int v : values) sum += v;
    int avg = sum / sample_count;

    return calculate_percentage(avg);
}

int Battery::read_adc_value() {
    uint8_t config[3] = {0x01, 0xC3, 0x83};  // 설정 레지스터
    if (write(file, config, 3) != 3) {
        std::cerr << "I2C 쓰기 실패\n";
        return -1; // 오류 발생시 -1 리턴
    }

    uint8_t pointer[1] = {0x00};
    if (write(file, pointer, 1) != 1) {
        std::cerr << "I2C 쓰기 실패\n";
        return -1; // 오류 발생시 -1 리턴
    }

    usleep(100000);

    uint8_t data[2];
    if (read(file, data, 2) != 2) {
        std::cerr << "I2C 읽기 실패\n";
        return -1; // 오류 발생시 -1 리턴
    }

    return (data[0] << 8) | data[1];
}
