# Shoepernoma

<img src="https://github.com/user-attachments/assets/bc0d3222-867b-44f1-ad89-bd9823baf61f" name="roscar" width="70%">

매장 연동형 로봇 기반 신발 피킹 시스템​

**addinedu 8기 파이널 프로젝트 3조**


## **Secondary Wave**팀 구성

| 역할 | 이름   | 주요 업무 요약 |
|------|--------|----------------------------------------------------------|
| 팀장 | 김연우 | - MainService 구조화 및 ROS2 통신 처리<br>- Task 처리 로직 및 ROS Action 연동<br>- 로봇 암 제어 및 rack/cart controller 개발<br>- Manager GUI 통신 구조 개선<br>- AI 모듈/DB 통합 및 시스템 리팩토링 |
| 팀원 | 나덕윤 | - YOLO 기반 AI 모델 학습 및 라벨링<br>- 객체 인식 결과 MainServer 연동<br>- Manager GUI 상 로봇 상태 표시 구현<br>- 로그인 및 UI 흐름 설계 |
| 팀원 | 남상기 | - 자율주행 기능 (TEB, 경로 계획, Waypoint)<br>- IMU, LiDAR, 초음파 센서 통신 처리<br>- ArUco 기반 절대 위치 추정<br>- Domain Bridge 및 통신 구조 설계 |
| 팀원 | 심채훈 | - SLAM 및 Navigation 환경 설정<br>- ArUco 기반 위치 보정<br>- Battery/SSID 센서 퍼블리싱 및 DB 연동<br>- UDP 기반 영상 송신<br>- GUI 기본 기능 구현 |


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

![System Architecture](https://github.com/user-attachments/assets/730f694e-7c41-4ea8-891b-623ae14bacbb)

---

## 기본주행

![Basic_driving](https://github.com/user-attachments/assets/944b3f10-5b37-466e-9ef4-fd37d4a90237)

---

## 정밀정차

![Precise stop](https://github.com/user-attachments/assets/2ecbc1c4-c21e-4b37-baad-ce581e4ed49f)
---

## 집품확인 (micro-ros)

---

## Dashboard

![Dashboard Image 1](https://github.com/user-attachments/assets/e2204ab3-1947-4924-a7ae-be2521c85c91)

![Dashboard Image 2](https://github.com/user-attachments/assets/1f6ed6ce-8fd7-4b66-a0e9-99cd54573325)

---
