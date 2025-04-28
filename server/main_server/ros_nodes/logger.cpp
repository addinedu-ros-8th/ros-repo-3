// logger.cpp
#include <iostream>
#include <fstream>
#include <ctime>
#include <mutex>

std::mutex log_mutex;
std::ofstream log_file("server_log.txt", std::ios::app);

void log(const std::string& level, const std::string& message) {
    std::lock_guard<std::mutex> guard(log_mutex);

    if (!log_file.is_open()) {
        std::cerr << "Failed to open log file!" << std::endl;
        return;
    }

    // 현재 시간 구하기
    std::time_t now = std::time(nullptr);
    std::tm* local_time = std::localtime(&now);

    char time_buffer[20];
    std::strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", local_time);

    log_file << "[" << time_buffer << "] [" << level << "] " << message << std::endl;
    log_file.flush();
}

void log_info(const std::string& message) {
    log("INFO", message);
}

void log_warning(const std::string& message) {
    log("WARNING", message);
}

void log_error(const std::string& message) {
    log("ERROR", message);
}
