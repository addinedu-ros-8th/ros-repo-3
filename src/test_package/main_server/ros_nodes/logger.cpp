#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
class Logger {
public:
    Logger(const std::string &filename) {
        log_file_.open(filename, std::ios::app);
    }
    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }
    void log(const std::string &message) {
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        log_file_ << std::ctime(&now) << " - " << message << std::endl;
    }
private:
    std::ofstream log_file_;
};
int main() {
    Logger logger("server.log");
    logger.log("Logger initialized.");
    logger.log("Received message from sensor.");
    return 0;
}