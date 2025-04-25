#include <iostream>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERVER_IP "192.168.0.100"  // 수신측 IP 주소
#define SERVER_PORT 5005          // 수신측 포트
#define ROBOT_ID 1                // 전송할 로봇 ID

int main() {
    // UDP 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return -1;
    }

    // 서버 주소 설정
    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    // 카메라 열기
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera." << std::endl;
        return -1;
    }

    std::vector<uchar> buf;
    std::vector<int> param = {cv::IMWRITE_JPEG_QUALITY, 50}; // 압축 품질 0~100

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        // JPEG로 압축
        buf.clear();
        cv::imencode(".jpg", frame, buf, param);

        // 데이터 구성: [robot_id(4 bytes) + JPEG 바이너리]
        std::vector<uchar> packet;
        packet.resize(sizeof(int) + buf.size());

        // robot_id 추가
        int rid = htonl(ROBOT_ID); // 네트워크 바이트 순서
        std::memcpy(packet.data(), &rid, sizeof(int));

        // JPEG 이미지 데이터 추가
        std::memcpy(packet.data() + sizeof(int), buf.data(), buf.size());

        // 전송
        ssize_t sent = sendto(sock, packet.data(), packet.size(), 0,
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            perror("sendto failed");
        }

        // 전송 속도 조절 (옵션)
        cv::waitKey(50); // 약 20 FPS
    }

    close(sock);
    return 0;
}
