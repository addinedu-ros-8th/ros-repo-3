# Shoepernoma

<img src="https://github.com/user-attachments/assets/bc0d3222-867b-44f1-ad89-bd9823baf61f" name="roscar" width="70%">

매장 연동형 로봇 기반 신발 피킹 시스템​

**addinedu 8기 파이널 프로젝트 3조**


## **Secondary Wave**팀 구성

| 역할   | 이름       |
|--------|------------|
| 팀장   | 김연우     |
| 팀원   | 나덕윤     |
| 팀원   | 남상기     |
| 팀원   | 심채훈     |

---

로봇이름: ROScar

로봇그룹: ROScars(로스카스 🐷)
# Shoepernoma: 자율주행 로봇 기반 신발 피킹 및 전달 시스템

매장 연동형 자율주행 로봇을 활용해 신발 상자를 자동으로 피킹(Picking)하고 전달하는 스마트 물류 솔루션입니다.

---

## 프로젝트 배경 및 목표

- **노동력 부족 및 인건비 상승**으로 오프라인 매장의 물류 처리 효율화 필요
- **기존 수작업 프로세스 한계**: 재고 처리 속도 저하, 배송 오류, 응대 지연 문제 발생
- **목표**: ROS2 기반 자율주행 로봇과 AI 영상 처리, GUI 연동을 통해 매장 내 물류 흐름 최적화 및 피킹 정확도 향상

---

## 핵심 기술 스택


| 분류             | 기술 요소 |
|------------------|------------------------------------------------------------------|
| 로봇 미들웨어  | ![ROS 2](https://img.shields.io/badge/ROS2-iron-blue?logo=ros) ![Nav2](https://img.shields.io/badge/Nav2-robot--navigation-blue) ![TF2](https://img.shields.io/badge/TF2-transform--library-lightgrey) |
| SLAM & 위치추정 | ![Cartographer](https://img.shields.io/badge/Cartographer-SLAM-orange) ![ArUco](https://img.shields.io/badge/ArUco-marker%20tracking-lightblue) |
| 센서 융합      | ![LiDAR](https://img.shields.io/badge/LiDAR-2D-green) ![IMU](https://img.shields.io/badge/IMU-inertial-yellow) ![Ultrasonic](https://img.shields.io/badge/Ultrasonic-distance-lightgrey) ![Camera](https://img.shields.io/badge/Camera-vision-red) |
| 객체 인식      | ![OpenCV](https://img.shields.io/badge/OpenCV-computer--vision-blue?logo=opencv) ![YOLO](https://img.shields.io/badge/YOLOv5-object%20detection-red) |
| 영상 전송      | ![UDP](https://img.shields.io/badge/UDP-streaming-darkblue) |
| GUI           | ![PyQt6](https://img.shields.io/badge/PyQt6-GUI-lightgreen) |
| 서버 & 네트워크 | ![Python](https://img.shields.io/badge/Python-3.12-blue?logo=python) ![TCP](https://img.shields.io/badge/TCP-protocol-yellow) ![MySQL](https://img.shields.io/badge/MySQL-database-blue?logo=mysql) |
| 협업 도구      | ![Git](https://img.shields.io/badge/Git-version%20control-orange?logo=git) ![Confluence](https://img.shields.io/badge/Confluence-docs-blue?logo=confluence) ![Jira](https://img.shields.io/badge/Jira-task%20mgmt-blue?logo=jira) |


---

## 시스템 아키텍처

1. **Embedded Controller** (`mobile_controller/`)
   - 센서 데이터 수집 및 액추에이터 제어
2. **ROS2 Middleware**
   - Roscar 노드들 (`shared_interfaces/msg`, `server/ros_nodes`)
   - Topic/Service/Action 기반 통신
3. **AI 영상 서버** (`server/ai_server/`)
   - 객체 탐지 및 트래킹 모듈
4. **메인 서버** (`server/main_server/`)
   - TCP/UDP 데이터 처리, DB 저장, 작업 할당 로직
5. **GUI & Dashboard** (`viewer/`)
   - **Staff Interface**: QR 코드 스캔, 재고 조회, 배송 요청/취소
   - **Manager Dashboard**: 실시간 위치 추적 및 상태 모니터링

---

## 🚀 설치 및 실행 방법

