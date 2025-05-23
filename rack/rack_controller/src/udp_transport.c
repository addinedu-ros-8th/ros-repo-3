#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <esp_system.h>

int sock = -1;
struct sockaddr_in remote_addr;

bool esp32_udp_open(struct uxrCustomTransport * transport) {
    (void) transport;
    printf("Entering esp32_udp_open\n");

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("❌ socket creation failed, errno: %d\n", errno);
        return false;
    }

    printf("✅ UDP socket opened, sock: %d\n", sock);
    return true;
}

bool esp32_udp_close(struct uxrCustomTransport * transport) {
    (void) transport;
    if (sock >= 0) {
        close(sock);
        printf("✅ UDP socket closed, sock: %d\n", sock);
        sock = -1;
    }
    return true;
}

size_t esp32_udp_write(struct uxrCustomTransport* transport,
                       const uint8_t* buf,
                       size_t len,
                       uint8_t* err) {
    if (sock < 0) {
        printf("❌ write failed: invalid socket\n");
        return 0;
    }
    int result = sendto(sock, buf, len, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
    printf("📤 sendto() result: %d (len: %d), errno: %d\n", result, len, errno);
    return result < 0 ? 0 : result;
}

size_t esp32_udp_read(struct uxrCustomTransport* transport,
                      uint8_t* buf,
                      size_t len,
                      int timeout,
                      uint8_t* err) {
    if (sock < 0) {
        printf("❌ read failed: invalid socket\n");
        return 0;
    }
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);
    int ret = recvfrom(sock, buf, len, 0, (struct sockaddr*)&from_addr, &from_len);
    printf("📥 recvfrom() result: %d, errno: %d\n", ret, errno);
    return ret < 0 ? 0 : ret;
}

bool set_microros_udp_transports(const char* agent_ip, uint16_t agent_port) {
    printf("Free heap before UDP setup: %u\n", esp_get_free_heap_size());
    printf("Setting up UDP transport for %s:%d\n", agent_ip, agent_port);

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(agent_port);
    remote_addr.sin_addr.s_addr = inet_addr(agent_ip);

    if (remote_addr.sin_addr.s_addr == INADDR_NONE) {
        printf("❌ Invalid agent IP address: %s\n", agent_ip);
        return false;
    }

    printf("✅ UDP transport configured: %s:%d\n", agent_ip, agent_port);
    printf("Calling rmw_uros_set_custom_transport\n");

    bool ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        esp32_udp_open,
        esp32_udp_close,
        esp32_udp_write,
        esp32_udp_read
    );
    if (!ret) {
        printf("❌ rmw_uros_set_custom_transport failed\n");
        if (sock >= 0) {
            close(sock);
            sock = -1;
        }
    } else {
        printf("✅ rmw_uros_set_custom_transport succeeded\n");
    }

    printf("Free heap after UDP setup: %u\n", esp_get_free_heap_size());
    return ret;
}