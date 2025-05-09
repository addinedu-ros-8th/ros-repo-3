#ifndef IMU_SENSOR_HPP_
#define IMU_SENSOR_HPP_

#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include "kalman_filter.hpp"

#define MAG_ADDR 0x0C

class ICM20948 {
public:
    ICM20948(const char* device = "/dev/i2c-1", int addr = 0x68);
    ~ICM20948();


    void initialize();
    void read_sensor(float& ax, float& ay, float& az,
                     float& gx, float& gy, float& gz,
                     float& mx, float& my, float& mz);

private:
    int file_;
    const char* device_;
    int addr_;

    KalmanFilter kalman_acc_x = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);
    KalmanFilter kalman_acc_y = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);
    KalmanFilter kalman_acc_z = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);

    KalmanFilter kalman_gyro_x = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);
    KalmanFilter kalman_gyro_y = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);
    KalmanFilter kalman_gyro_z = KalmanFilter(0.01f, 0.1f, 0.1f, 0.0f);
    void open_i2c();
    void set_bank(uint8_t bank);
    void write_reg(uint8_t bank, uint8_t reg, uint8_t data);
    uint8_t read_reg(uint8_t bank, uint8_t reg);
    int16_t read_word(uint8_t bank, uint8_t reg);

    void init_icm20948();  // 현재 사용 안 하지만 보존
    void enable_mag();
    void read_mag_data(float& mx, float& my, float& mz);
};

#endif // IMU_SENSOR_HPP_
