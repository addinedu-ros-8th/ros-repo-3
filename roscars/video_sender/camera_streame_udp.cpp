#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERVER_IP   "192.168.0.100"  // AI 서버 IP 주소
#define SERVER_PORT 5005            // 포트 번호
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FPS         30
#define MAX_UDP_PACKET_SIZE 65507   // UDP 최대 payload (IPv4 기준)
#define MJPEG_STREAM "tcp://127.0.0.1:8888"

int main() {
    // 카메라 캡처 준비
    cv::VideoCapture cap(MJPEG_STREAM, cv::CAP_FFMPEG);

    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, FPS);

    // UDP 소켓 생성
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("소켓 생성 실패");
        return -1;
    }

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    uint32_t frame_number = 0;
    const int header_size = sizeof(uint32_t);  // 프레임 번호용 4바이트

    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            // std::cerr << "빈 프레임을 수신했습니다." << std::endl;
            continue;
        }

        // JPEG 압축
        std::vector<uchar> encoded;
        std::vector<int> encode_param = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame, encoded, encode_param);

        // UDP 전송 가능한 크기 확인
        if (encoded.size() + header_size > MAX_UDP_PACKET_SIZE) {
            std::cerr << "프레임 크기가 너무 큽니다. 전송 생략. (" << encoded.size() << " bytes)" << std::endl;
            continue;
        }

        // 전송 버퍼 구성 (frame_number + jpeg data)
        std::vector<uchar> send_buffer(header_size + encoded.size());
        std::memcpy(send_buffer.data(), &frame_number, header_size); // 앞에 프레임 번호 추가
        std::memcpy(send_buffer.data() + header_size, encoded.data(), encoded.size());

        // UDP 전송
        ssize_t sent = sendto(sock, send_buffer.data(), send_buffer.size(), 0,
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            perror("UDP 전송 실패");
        } else {
            std::cout << "프레임 #" << frame_number << " 전송 (" << sent << " bytes)" << std::endl;
        }

        frame_number++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / FPS));
    }

    close(sock);
    return 0;
}
