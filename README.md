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

- **로봇 미들웨어**: ROS 2 (Nav2, TF2)
- **SLAM & 위치추정**: Cartographer SLAM + ArUco 마커 기반 초기 포즈 보정
- **센서 융합**: LiDAR, IMU, 초음파 센서, 카메라
- **객체 인식**: OpenCV, YOLO 계열 모델
- **영상 전송**: UDP 기반 실시간 스트리밍
- **GUI**: PyQt6 (Staff Interface, Manager Dashboard)
- **서버 & 네트워크**: Python TCP 서버, 바이너리 프로토콜, MySQL 데이터베이스
- **협업 도구**: Git, Confluence, Jira

---

##  리포지터리 구조
```
ros-repo-3/
├── docs/ # 설계 문서 (아키텍처, 인터페이스, ERD 등)
├── shared_interfaces/ # ROS 메시지·서비스 정의
├── mobile_controller/ # 임베디드(C++) & ROS 노드 (센서, 액추에이터)
├── server/ # AI 서버(object detection) & 메인 서버(통신·DB·ROS 노드)
├── viewer/ # PyQt6 기반 Staff GUI 및 Manager Dashboard
├── requirements.txt # Python 패키지 의존성
├── roscars.sql # 초기 DB 스키마 덤프
└── README.md # 프로젝트 개요 및 실행 가이드
```

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

