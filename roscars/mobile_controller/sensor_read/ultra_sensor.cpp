#include "ultra_sensor.hpp"
#include <chrono>
#include <gpiod.h>
#include <cmath>
#include <stdexcept>  // 추가: std::runtime_error를 사용하려면 필요
#include <thread>     // 추가: std::this_thread::sleep_for를 사용하려면 필요

UltraSensor::UltraSensor() {
    chip = gpiod_chip_open_by_name("gpiochip4"); // GPIO 23, 24 = BCM 기준 chip 4
    if (!chip) {
        throw std::runtime_error("Failed to open gpiochip4");
    }

    trig_line = gpiod_chip_get_line(chip, 23); // TRIG = GPIO 23
    echo_line = gpiod_chip_get_line(chip, 24); // ECHO = GPIO 24

    gpiod_line_request_output(trig_line, "ultra_trig", 0);
    gpiod_line_request_input(echo_line, "ultra_echo");
}

UltraSensor::~UltraSensor() {
    gpiod_line_release(trig_line);
    gpiod_line_release(echo_line);
    gpiod_chip_close(chip);
}

float UltraSensor::read_distance() {
    // 10us high trigger pulse
    gpiod_line_set_value(trig_line, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));  // 10us 대기
    gpiod_line_set_value(trig_line, 0);

    // Wait for echo start
    auto start = std::chrono::high_resolution_clock::now();
    while (gpiod_line_get_value(echo_line) == 0) {
        start = std::chrono::high_resolution_clock::now();
    }

    // Wait for echo end
    auto end = start;
    while (gpiod_line_get_value(echo_line) == 1) {
        end = std::chrono::high_resolution_clock::now();
    }

    std::chrono::duration<float> duration = end - start;
    float distance = (duration.count() * 34300.0) / 2.0; // cm 단위

    // 소수점 2자리까지 반올림하여 저장
    distance = std::round(distance * 100.0) / 100.0;  // 소수점 2자리 반올림
    return distance;
}
