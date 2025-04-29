#ifndef MOBILE_CONTROLLER__UTILS_HPP_
#define MOBILE_CONTROLLER__UTILS_HPP_

#include <string>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace mobile_controller
{

inline std::string get_ip_address(const std::string& interface_name)
{
    struct ifaddrs *interfaces = nullptr;
    struct ifaddrs *ifa = nullptr;
    void *tmp_addr_ptr = nullptr;
    std::string ip_address = "";

    getifaddrs(&interfaces);
    for (ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) { // IPv4만
            if (interface_name == ifa->ifa_name) {
                tmp_addr_ptr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                char address_buffer[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, tmp_addr_ptr, address_buffer, INET_ADDRSTRLEN);
                ip_address = address_buffer;
                break;
            }
        }
    }
    if (interfaces) freeifaddrs(interfaces);
    return ip_address;
}

inline std::string get_hostname()
{
    char hostname[1024];
    gethostname(hostname, 1024);
    return std::string(hostname);
}

}  // namespace mobile_controller

#endif  // MOBILE_CONTROLLER__UTILS_HPP_
