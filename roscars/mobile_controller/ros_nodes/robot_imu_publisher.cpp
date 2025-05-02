#include "rclcpp/rclcpp.hpp"
#include "shared_interfaces/msg/imu_status.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>

#define MAG_ADDR 0x0C

std::string get_ap_ssid() {
    std::ifstream hostapd_file("/etc/hostapd/hostapd.conf");
    std::string line;
    while (std::getline(hostapd_file, line)) {
        if (line.find("ssid=") == 0) {
            return line.substr(5);
        }
    }
    return "UNKNOWN_SSID";
}

class ICM20948Publisher : public rclcpp::Node {
public:
    ICM20948Publisher() : Node("imu_publisher") {
        publisher_ = this->create_publisher<shared_interfaces::msg::ImuStatus>("/robot/sensor/imu", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ICM20948Publisher::publish_imu, this));

        open_i2c();
        init_icm20948();
    }

    ~ICM20948Publisher() {
        close(file_);
    }

private:
    const char *device_ = "/dev/i2c-1";
    const int addr_ = 0x68;
    int file_;
    rclcpp::Publisher<shared_interfaces::msg::ImuStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void open_i2c() {
        file_ = open(device_, O_RDWR);
        if (file_ < 0 || ioctl(file_, I2C_SLAVE, addr_) < 0) {
            RCLCPP_FATAL(this->get_logger(), "I2C 연결 실패: %s", device_);
            std::exit(1);
        }
    }

    void write_reg(uint8_t bank, uint8_t reg, uint8_t data) {
        set_bank(bank);
        uint8_t buf[2] = {reg, data};
        if (write(file_, buf, 2) != 2) {
            RCLCPP_WARN(this->get_logger(), "레지스터 쓰기 실패: 0x%02X", reg);
        }
        usleep(1000);
    }

    uint8_t read_reg(uint8_t bank, uint8_t reg) {
        set_bank(bank);
        uint8_t cmd = reg;
        write(file_, &cmd, 1);
        uint8_t val = 0;
        read(file_, &val, 1);
        return val;
    }

    int16_t read_word(uint8_t bank, uint8_t reg) {
        set_bank(bank);
        uint8_t cmd = reg;
        write(file_, &cmd, 1);
        uint8_t data[2] = {0};
        read(file_, data, 2);
        return static_cast<int16_t>((data[0] << 8) | data[1]);
    }

    void set_bank(uint8_t bank) {
        uint8_t buf[2] = {0x7F, static_cast<uint8_t>(bank << 4)};
        write(file_, buf, 2);
        usleep(1000);
    }

    void init_icm20948() {
        write_reg(0, 0x06, 0x01);  // PWR_MGMT_1
        usleep(10000);
        write_reg(0, 0x07, 0x00);  // PWR_MGMT_2
        usleep(10000);
        enable_mag();
    }

    void enable_mag() {
        write_reg(0, 0x06, 0x01); // Clock
        write_reg(0, 0x03, 0x01); // Enable I2C Master
        usleep(10000);

        write_reg(3, 0x03, MAG_ADDR);    // I2C_SLV0_ADDR
        write_reg(3, 0x04, 0x0A);        // CNTL2
        write_reg(3, 0x05, 0x01);        // Single Measurement Mode
        write_reg(3, 0x06, 0x81);        // Enable + 1 byte
        usleep(10000);
    }

    void read_mag_data(float& mx, float& my, float& mz) {
        write_reg(3, 0x03, MAG_ADDR | 0x80);  // Read mode
        write_reg(3, 0x04, 0x10);            // Start from ST1
        write_reg(3, 0x05, 9);               // 9 bytes
        write_reg(3, 0x06, 0x80 | 9);        // Enable
        usleep(10000);

        set_bank(0);
        uint8_t cmd = 0x3B;
        write(file_, &cmd, 1);
        uint8_t data[9] = {0};
        read(file_, data, 9);

        int16_t raw_x = (data[1] << 8) | data[0];
        int16_t raw_y = (data[3] << 8) | data[2];
        int16_t raw_z = (data[5] << 8) | data[4];

        mx = raw_x * 0.15;  // μT
        my = raw_y * 0.15;
        mz = raw_z * 0.15;
    }

    void publish_imu() {
        int16_t ax = read_word(0, 0x2D);
        int16_t ay = read_word(0, 0x2F);
        int16_t az = read_word(0, 0x31);
        int16_t gx = read_word(0, 0x33);
        int16_t gy = read_word(0, 0x35);
        int16_t gz = read_word(0, 0x37);

        float mx = 0.0, my = 0.0, mz = 0.0;
        read_mag_data(mx, my, mz);

        shared_interfaces::msg::ImuStatus imu_msg;
        imu_msg.accel_x = ax / 2048.0;
        imu_msg.accel_y = ay / 2048.0;
        imu_msg.accel_z = az / 2048.0;

        imu_msg.gyro_x = gx / 16.4 * M_PI / 180.0;
        imu_msg.gyro_y = gy / 16.4 * M_PI / 180.0;
        imu_msg.gyro_z = gz / 16.4 * M_PI / 180.0;

        imu_msg.mag_x = mx;
        imu_msg.mag_y = my;
        imu_msg.mag_z = mz;

        imu_msg.roscar_name = get_ap_ssid();

        publisher_->publish(imu_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICM20948Publisher>());
    rclcpp::shutdown();
    return 0;
}
