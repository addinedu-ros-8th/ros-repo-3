#include "imu_sensor.hpp"
#include "kalman_filter.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cmath>

ICM20948::ICM20948(const char* device, int addr) : device_(device), addr_(addr) {
    open_i2c();
}

ICM20948::~ICM20948() {
    close(file_);
}

void ICM20948::open_i2c() {
    file_ = open(device_, O_RDWR);
    if (file_ < 0 || ioctl(file_, I2C_SLAVE, addr_) < 0) {
        std::cerr << "I2C 연결 실패: " << device_ << std::endl;
        std::exit(1);
    }
}

void ICM20948::set_bank(uint8_t bank) {
    uint8_t buf[2] = {0x7F, static_cast<uint8_t>(bank << 4)};
    write(file_, buf, 2);
    usleep(1000);
}

void ICM20948::write_reg(uint8_t bank, uint8_t reg, uint8_t data) {
    set_bank(bank);
    uint8_t buf[2] = {reg, data};
    if (write(file_, buf, 2) != 2) {
        std::cerr << "레지스터 쓰기 실패: 0x" << std::hex << static_cast<int>(reg) << std::endl;
    }
    usleep(1000);
}

uint8_t ICM20948::read_reg(uint8_t bank, uint8_t reg) {
    set_bank(bank);
    uint8_t cmd = reg;
    write(file_, &cmd, 1);
    uint8_t val = 0;
    read(file_, &val, 1);
    return val;
}

int16_t ICM20948::read_word(uint8_t bank, uint8_t reg) {
    set_bank(bank);
    uint8_t cmd = reg;
    write(file_, &cmd, 1);
    uint8_t data[2] = {0};
    read(file_, data, 2);
    return static_cast<int16_t>((data[0] << 8) | data[1]);
}

void ICM20948::initialize() {
    write_reg(0, 0x06, 0x01);  // PWR_MGMT_1
    usleep(10000);
    write_reg(0, 0x07, 0x00);  // PWR_MGMT_2
    usleep(10000);
    enable_mag();
}

void ICM20948::enable_mag() {
    write_reg(0, 0x06, 0x01); // Clock
    write_reg(0, 0x03, 0x01); // Enable I2C Master
    usleep(10000);

    write_reg(3, 0x03, MAG_ADDR);    // I2C_SLV0_ADDR
    write_reg(3, 0x04, 0x0A);        // CNTL2
    write_reg(3, 0x05, 0x01);        // Single Measurement Mode
    write_reg(3, 0x06, 0x81);        // Enable + 1 byte
    usleep(10000);
}

void ICM20948::read_sensor(float& ax, float& ay, float& az,
        float& gx, float& gy, float& gz,
        float& mx, float& my, float& mz) {
    int16_t raw_ax = read_word(0, 0x2D);
    int16_t raw_ay = read_word(0, 0x2F);
    int16_t raw_az = read_word(0, 0x31);
    int16_t raw_gx = read_word(0, 0x33);
    int16_t raw_gy = read_word(0, 0x35);
    int16_t raw_gz = read_word(0, 0x37);

    float temp_ax = raw_ax / 2048.0;
    float temp_ay = raw_ay / 2048.0;
    float temp_az = raw_az / 2048.0;

    float temp_gx = raw_gx / 16.4 * M_PI / 180.0;
    float temp_gy = raw_gy / 16.4 * M_PI / 180.0;
    float temp_gz = raw_gz / 16.4 * M_PI / 180.0;

    ax = kalman_acc_x.update(temp_ax);
    ay = kalman_acc_y.update(temp_ay);
    az = kalman_acc_z.update(temp_az);

    gx = kalman_gyro_x.update(temp_gx);
    gy = kalman_gyro_y.update(temp_gy);
    gz = kalman_gyro_z.update(temp_gz);

    read_mag_data(mx, my, mz);
}

void ICM20948::read_mag_data(float& mx, float& my, float& mz) {
    write_reg(3, 0x03, MAG_ADDR | 0x80);
    write_reg(3, 0x04, 0x10);
    write_reg(3, 0x05, 9);
    write_reg(3, 0x06, 0x80 | 9);
    usleep(10000);

    set_bank(0);
    uint8_t cmd = 0x3B;
    write(file_, &cmd, 1);
    uint8_t data[9] = {0};
    read(file_, data, 9);

    int16_t raw_x = (data[1] << 8) | data[0];
    int16_t raw_y = (data[3] << 8) | data[2];
    int16_t raw_z = (data[5] << 8) | data[4];

    mx = raw_x * 0.15;
    my = raw_y * 0.15;
    mz = raw_z * 0.15;
}
