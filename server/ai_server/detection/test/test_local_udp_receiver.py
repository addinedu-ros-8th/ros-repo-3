#include <opencv2/opencv.hpp>
#include <zlib.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERVER_IP   "192.168.0.100"  // 수신 서버 IP
#define SERVER_PORT 9999
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 360
#define FPS         30

int main() {
    cv::VideoCapture cap(0);  // 카메라 열기

    if (!cap.isOpened()) {
        std::cerr << "카메라를 열 수 없습니다." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FPS, FPS);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("소켓 생성 실패");
        return -1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "빈 프레임입니다." << std::endl;
            continue;
        }

        // JPEG로 인코딩
        std::vector<uchar> jpeg_buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", frame, jpeg_buf, params)) {
            std::cerr << "JPEG 인코딩 실패" << std::endl;
            continue;
        }

        // zlib으로 압축
        uLongf compressed_size = compressBound(jpeg_buf.size());
        std::vector<uchar> compressed_buf(compressed_size);

        int res = compress2(compressed_buf.data(), &compressed_size,
                            jpeg_buf.data(), jpeg_buf.size(), Z_BEST_SPEED);

        if (res != Z_OK) {
            std::cerr << "압축 실패: " << res << std::endl;
            continue;
        }

        // UDP 전송
        ssize_t sent = sendto(sock, compressed_buf.data(), compressed_size, 0,
                              (sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            perror("UDP 전송 실패");
        } else {
            std::cout << "프레임 전송 완료 (" << sent << " bytes)" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / FPS));
    }

    close(sock);
    return 0;
}
