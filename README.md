# Shoepernoma

> *(KDT) ROS2와 인공지능을 활용한 자율주행 로봇 개발자 양성과정 8기* 교육과정 내에서 진행된 파이널 과제입니다. 3팀 **Secondary Wave**는 본 프로젝트로 최종 발표회에서 최우수상(1위)을 수상하였습니다.

<img src="https://github.com/user-attachments/assets/bc0d3222-867b-44f1-ad89-bd9823baf61f" name="roscar" width="70%">

매장 연동형 자율주행 로봇을 활용해 신발 상자를 자동으로 피킹(Picking)하고 전달하는 스마트 물류 솔루션입니다.
- 오프라인 매장 내 물류 처리 자동화 및 효율화
- ROS2 기반 자율주행 로봇 개발 및 AI 영상 처리 기술 적용
- GUI 연동으로 실시간 재고 관리 및 피킹 요청 처리

LiDAR+IMU 기반 2D SLAM(Cartographer)으로 지도를 작성하고 자율주행에는 Nav2이 사용되었습니다. \
OpenCV ArUco Marker를 활용해 주행 초기 위치 보정과 근접 정렬·도킹 기능을 구현했습니다. \
YOLOv8n 모델로 신발 상자 객체 인식을 수행하고, Python 기반 GUI로 로봇 상태 및 재고 현황을 실시간으로 모니터링할 수 있습니다.

<table>
	<tr>
		<td>
			<img src="https://github.com/user-attachments/assets/4585d2f8-7e83-4e42-85a2-2f14ef8771d6" alt="Shopernoma_판넬(가로)"/>
		</td>
		<td>
			<img src="https://github.com/user-attachments/assets/d3e87516-378f-4bba-8837-b080d74d7d17" alt="Shopernoma_판넬(세로)"/>
		</td>
	</tr>
</table>


## 프로젝트 개요

기간: 2025.04.09 ~ 2025.05.27 (7주)
팀명: Secondary Wave
로봇이름: ROScar
로봇그룹: ROScars(로스카츠 🐷)

| 역할 | 이름   | 주요 업무 요약 |
|------|--------|----------------------------------------------------------|
| 팀장 | 김연우 | - MainService 구조화 및 ROS2 통신 처리<br/>- Task 처리 로직 및 ROS Action 연동<br/>- 로봇 암 제어 및 rack·cart controller 개발<br/>- Manager GUI 통신 구조 개선<br/>- AI 모듈·DB 통합 및 시스템 리팩토링 |
| 팀원 | 나덕윤 | - YOLO 기반 AI 모델 학습 및 라벨링<br/>- 객체 인식 결과 MainServer 연동<br/>- Manager GUI 상 로봇 상태 표시 구현<br/>- 로그인 및 UI 흐름 설계 |
| 팀원 | 남상기 | - 자율주행 기능 (TEB, 경로 계획, Waypoint)<br/>- IMU, LiDAR, 초음파 센서 통신 처리<br/>- ArUco 기반 절대 위치 추정<br/>- Domain Bridge 및 통신 구조 설계 |
| 팀원 | 심채훈 | - SLAM 및 Navigation 환경 설정<br/>- ArUco 기반 위치 보정<br/>- Battery·SSID 센서 퍼블리싱 및 DB 연동<br/>- UDP 기반 영상 송신<br/>- GUI 기본 기능 구현 |

### 주요 주행 기술

| 기능 | 기술 |
| --- | --- |
| 지도·SLAM | LiDAR+IMU 2D SLAM (Cartographer) |
| 초기 위치 보정 | ArUco 마커 기반 `/initialpose` 보정 (OpenCV ArUco + Cartographer trajectory restart) |
| 자율 주행·경로 계획 | Nav2 (NavFn 전역 플래너 + DWB 로컬 플래너 + 코스트맵) |
| 웨이포인트 주행 | Nav2 FollowWaypoints 액션 |
| 근접 정렬·도킹 | ArUco 기반 PID 정렬 |
| 시각화 | RViz (SLAM·주행 뷰 분리) |
| IMU 노이즈 보정 | 칼만 필터 기반 안정화 |


### 전체 기술 스택

| 분류             | 기술 요소 |
|------------------|------------------------------------------------------------------|
| 로봇 미들웨어  | ROS2, Nav2, TF2 |
| 언어            | Python, C++ |
| SLAM & 위치추정 | Cartographer SLAM, ArUco, Lifecycle Node |
| 센서 융합      | LiDAR, IMU, Ultrasonic, Camera, Micro-ROS |
| 객체 인식      | OpenCV, YOLOv8n |
| GUI           | PyQt6 |
| 통신       | TCP, UDP(영상 전송), ROS2(Action·Service·Topic) |
| 데이터베이스 | MySQL, ORM |
| 협업 도구      | Git, Slack, Jira, Confluence |


# 프로젝트 결과 및 자료

<img src="https://github.com/user-attachments/assets/a4f724f7-fc46-40b1-aa6f-de717f45b5c9" alt="통합영상" />


## 구현

하드웨어와 함께 제공된 ROS2 워크스페이스 [pinky_violet](https://github.com/pinklab-art/pinky_violet)에서 필요한 패키지를 Shopernoma에 맞게 수정하여 사용했습니다. \
주요 패키지로는 `pinky_cartographer`, `pinky_navigation`, `pinky_bringup`가 있습니다.

| 패키지명 | 패키지 기능 | 변경·적용 내용 |
| --- | --- | --- |
| `pinky_cartographer` | SLAM(Cartographer)으로 지도 작성 | 그대로 사용(기능·설정 변경 없음) |
| `pinky_navigation` | Nav2 주행 스택(플래너·컨트롤러·코스트맵) | 맵 파일 교체, `nav2_params.yaml` 튜닝(footprint·robot_radius 등) |
| `pinky_bringup` | 모터·배터리·센서 초기화 런치 | 모터·배터리 노드는 그대로 사용. <br/> 센서 및 카메라는 별도로 구현 |


## 주행 기술

<img src="https://github.com/user-attachments/assets/8ac805cf-48c8-403c-a4ba-a10233a89070" alt="전체동작-시도" />

<table>
<tr>
	<td>
		<img src="https://github.com/user-attachments/assets/c0c0d919-fbfb-4f89-a38d-6050ca014e61" alt="D7 주행LIVE-1" />
	</td>
	<td>
		<img src="https://github.com/user-attachments/assets/c521bb31-4f15-43dd-99be-5ec7d755dd20" alt="D7 주행LIVE-2" />
	</td>
</tr>
</table>

<br/>
<br/>
<br/>

**주행 시행착오(발전 과정)**

<!-- <img src="https://github.com/user-attachments/assets/66ac826f-af7f-4e7d-908f-de86adf598a7" alt="개발과정-주행시행착오-모음(2배속) mp4 readme" /> -->

<img src="https://github.com/user-attachments/assets/e018f741-157a-4f35-8925-623de0e39687" alt="개발과정-주행시행착오-4" />

<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/78fb52db-cc46-499f-945f-13979cf7bec8" alt="개발과정-주행시행착오-0" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/2e0f558b-5e99-4dd9-a21b-2c13d1497173" alt="개발과정-주행시행착오-1" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/ba83bd15-5962-4a8b-9b89-80c525fdbd1b" alt="개발과정-주행시행착오-2" />
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/3bcc8c67-af04-44ff-a130-faff6cc8e2e1" alt="개발과정-주행시행착오-3" />
    </td>
    <td>
			<img src="https://github.com/user-attachments/assets/e40162ff-62ba-4397-b2ef-9eadfb3a377c" alt="개발과정-주행시행착오-6" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/c14c964e-19fd-44bc-8200-4967e3445c25" alt="개발과정-주행시행착오-5" />
    </td>
  </tr>
</table>



### 1. 센서 입력 수집

<img width="330" height="230" alt="(판넬)센서퓨전" src="https://github.com/user-attachments/assets/728b8f28-83cb-4da3-bbae-24931b6597cd" />

- GUI에 사용되는 센서 및 상태 퍼블리셔는 C++로 구현하여 지연·부하를 줄이고 안정적인 타이밍을 확보하도록 함
  <!-- - 주행쪽에서도 C++로 구현한 데이터를 사용하고싶었으나, `pinky_bringup`에서 Cartographer와 Nav2 등과 같이 주행 부분에 연결된 데이터가 너무 많아 그대로 사용할 수밖에 없었음 (시간문제) -->
- 주행 관련 구현에는 Python으로 기존에 제공된 `pinky_bringup` 패키지의 노드를 그대로 사용
	- 제조사 SDK로 센서 값을 직접 읽기 위해 `mobile_controller`에 센서 퍼블리셔 노드 별도 구현 (기본 설정은 표준 ROS 드라이버)
		- IMU: ICM20948 <!-- (`/dev/i2c-1`, `0x68`) → `/roscar/sensor/imu` -->
		- LiDAR: SLLidar SDK  <!-- (`/dev/ttyAMA0`, 460800) → `/roscar/sensor/lidar` -->
		- 초음파: GPIO <!-- TRIG=23, ECHO=24 (`gpiod`) → `/roscar/sensor/ultra` -->
		- 배터리: ADS1115 <!-- (`0x48`) 샘플링 → `/roscar/status/battery` -->
  - 카메라
		<!-- - ROS 이미지 토픽 대신에 UDP 스트리밍 사용한 이유: 학습목적도 있었기에 ROS끼리 통신과 ROS와 ROS아닌 통신 간의 차이를 경험해보고자 함. -->
		- 메인 스레드에서 Picamera2로 직접 캡처
		- 프레임을 UDP로 스트리밍 전송하면서, 같은 프레임으로 ArUco 정렬(PID)을 수행
		- 정렬 결과 /cmd_vel 퍼블리시는 메인 루프에서 ArucoAlign.process_frame()이 직접 수행
		- ROS 콜백 처리 루프인 `rclpy.spin(align_node)`는 별도의 스레드로 분리해 메인 루프가 블로킹되지 않게 유지


<br/>
<br/>
<br/>

### 2. SLAM 및 지도 작성

<img width="500" alt="구현설명4-Cartographer" src="https://github.com/user-attachments/assets/8f66a981-cc10-4e74-8ca6-22b4b9d31674" />

- Cartographer(2D SLAM)으로 갈 수 있는 공간(흰색)과 갈 수 없는 공간(검정색)을 구분하여 `roscars_map.pgm`(흑백 이미지 파일) 지도를 작성
- LiDAR(`/scan`) + IMU(`/imu`) 융합으로 센서가 인식될 때마다 Cartographer가 `/map`, `/tf`를 사용해 자동으로 맵 업데이트
- 이때, 업데이트되면서 기존에 흑색인 부분이 희색으로 바뀌는 현상 발생 → 그림판으로 갈 수 없는 공간(검정색) 복원하여 최종 지도 작성
	<!-- - PGM은 흑·백 점유 지도이므로, 필요 시 수동 편집으로 장애물(검정) 영역 복구 가능 -->


<br/>
<br/>
<br/>


### 3. [Nav2] 자율주행

- 전역 플래너: NavFn (Dijkstra)
- 로컬 플래너: DWB

- `pinky_navigation`에서 Nav2 주행 스택 기능 사용하여 SLAM으로 작성한 지도를 적용하고 플래너, 컨트롤러, 코스트맵 등의 설정 조정
	- 맵 파일 교체와 `nav2_params.yaml` 튜닝(footprint·robot_radius 등)을 적용
	- costmap 구성 및 velocity_smoother 사용으로 주행 안정화
	- 전역 경로는 임의로 선정한 웨이포인트를 순서대로 NavFn이 계산

위와 같이 Dijkstra와 DWB 조합으로 최종 구현됐으며, 아래는 초기 목표였던 A*과 TEB 적용 시도 내용입니다. 

<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/103188dc-7850-4dad-a796-07485f2b6d24" alt="구현설명2-A*알고리즘" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/6055de4b-1547-44bf-8ace-688e986c80f4" alt="구현설명3-A*알고리즘" />
    </td>
  </tr>
</table>


- 초기 목표인 A*과 TEB는 아래와 같은 문제로 적용하지 못함
	- TEB는 당시에 Jazzy 버전의 사용 가능한 오픈소스가 없어 직접 구현했으나, 좌우로 움직이며 둠칫둠칫하는 등의 불안정한 주행 현상 발생
  - A*은 구현하여 nodes까지 구성·연결했으나, edges가 구성·연결이 완료되지 않아 실제 구현에는 적용되지 못함


<!-- 

- 맵 로드: `roscars/pinky_navigation/map/roscars_map.yaml`
- 설정: `roscars/pinky_navigation/params/nav2_params.yaml`


footprint는 로봇의 바닥면 윤곽을 2D 다각형(좌표 리스트)으로 정의한 것으로, 
	회피/경로 계획 시 로봇이 실제로 차지하는 영역을 반영함.
로봇이 주행 중에 벽에 너무 가까이 붙거나 급격히 방향을 바꾸는 현상이 발생하여 로봇의 크기를 높게 설정해 보완함.
robot_radius는 로봇을 원형으로 단순화했을 때의 반지름 값으로, 
	footprint 대신 단순 모델로 충돌 여유를 계산할 때 사용됨.

velocity_smoother는 가속도 제한 노드로, 로봇이 급격히 속도를 변경하지 않도록 줌. \
이를 통해 로봇의 움직임이 보다 부드럽고 안정적으로 변하며, 주행 중 흔들림이나 충격을 줄여줌.

costmap은 로봇 주변 공간을 비용 지도로 나타내는 2D 그리드 맵으로 각 셀에 로봇의 이동 여부를 나타내 장애물, 자유 공간, 위험 구역 등을 표시함. \
  장애물과 가까울수록 비용을 높게 매겨서, 경로 계획기가 안전한 공간을 우선으로 선택하도록 만든다.
  - 장애물 = 높은 비용
  - 통과 가능한 공간 = 낮은 비용
  - **장애물 주변에 완충 영역(Inflation)**을 만들어 충돌여유를 확보
Nav2는 이 costmap을 사용하여 경로 계획과 장애물 회피를 수행함.
  - global costmap: 전체 경로 계획용
  - local costmap: 주변 회피/미세 제어용 
	
	-->

<br/>
<br/>
<br/>

### 4. [FollowWaypoints] 웨이포인트 주행

<img width=600 alt="(판넬)ROS2NAV2" src="https://github.com/user-attachments/assets/41cfebd5-7c51-4621-af1a-14e4f25a9fb9" />

- 개발용으로 갈림길의 좌표 6개를 웨이포인트로 사전에 정의하여 주행 테스트
  - 좌표는 RViz에서 `2D Nav Goal`로 위치를 지정한 후에 `Publish Point`로 좌표 기록
- `FollowWaypoints` action으로 웨이포인트 순차 주행

<!-- 
- Nav2 `FollowWaypoints` 액션으로 웨이포인트 순차 주행
  - Nav2 waypoint_follower 서버 + C++ 클라이언트 구성
- `roscars/mobile_controller/nav_drive/waypoint_follower.cpp`에서 액션 클라이언트 구현
- A* 적용
  - `roscars/mobile_controller/nav_drive/astar.cpp`에 A* 구현
  - `waypoint_follower.cpp`에서 `astar.search(0, 5)`로 경로를 계산 후 웨이포인트 전송
  - `edges`가 비어 있어 그래프가 구성되지 않음
    - A* 호출은 실행되지만 유효한 최단 경로를 만들지 못할 수 있음 
-->

<br/>
<br/>
<br/>

### 5. [ArUco `/initialpose`] 초기 위치 보정

<img width=200 alt="(판넬)ARUCO마커초기포즈보정" src="https://github.com/user-attachments/assets/9f91b184-e2a1-470b-b98f-4e1ad64096f3" />

- ArUco 마커 ID를 map 좌표에 매핑해 사용 <!-- `roscars/aruco_mapper/aruco_marker_positions.json` -->
- 좌표 산정 흐름:
  - RViz Nav Goal·Publish Point로 좌표를 확인
  - ArUco 역인식으로 마커 좌표 추정 시도했으나 오차가 커서 중단(키보드 방향키 teleop로 이동하며 카메라로 마커 인식 → CSV 저장)
  - 카메라와 마커간의 거리를 줄자로 실제 길이를 측정하고 map 좌표계(m 단위)로 변환 좌표값을 보정
  - 최종 좌표값을 JSON에 key-value로 저장
- Picamera2 프레임에서 ArUco 검출 → `/initialpose` 퍼블리시
- 마커 방향 그룹(E·W·N·S)별 오프셋 보정 로직 포함
- Cartographer trajectory 재시작으로 초기 위치 반영

<!-- 
로컬라이저: 마커를 보고 현재 위치나 자세를 추정하는 노드 
- ArUco 로컬라이저는 마커 위치를 시각화용으로만 퍼블리시하며, 네비게이션에 직접 관여하지 않음. 
- 즉, 위치를 계산하긴 하되 이동 계획이나 위치 추정 흐름에 직접 반영하지 않음
-->

<!-- - yaw 각도: 
	- 기체나 물체가 수직축을 중심으로 좌우로 회전한 각도
	- 어느 방향을 바라보는지 나타내는 각도 (0°=동, 90°=북, 180°=서, 270°=남) -->

<!-- - 마커 방향 그룹(E·W·N·S):
	- ArUco 마커가 카메라 프레임 기준으로 어느 방향을 바라보는지에 따라 묶은 분류
	- 카메라 프레임 기준으로 마커 자세를 추정하면 마커가 바라보는 방향에 따라 위치 오차 패턴이 다르게 나타남
	- yaw 각도를 기준으로 동·서·남·북 네 그룹으로 나누고, 미리 측정한 오프셋을 더하거나 빼서 initialpose에 올리는 위치·각도 계산
  - 방향별로 오차 패턴이 달라서 각 그룹마다 보정값을 적용해야함
  - 결과적으로 같은 마커라도 방향에 맞는 보정을 거쳐 위치·각도 정확도가 높아짐 -->

<!-- 
- Cartographer의 trajectory:
	- 로봇의 위치·자세가 시간에 따라 이어진 경로(즉, 지금까지 어디를 어떻게 움직였는지 기록한 것)
	- 센서 데이터를 적분해서 추정
	- Cartographer는 보통 로컬 trajectory로 단기 추정을 만들고, 백엔드 최적화로 전역 trajectory를 보정하면서 지도를 갱신함 
-->

<br/>
<br/>
<br/>


### 6. [ArUco PID] 근접 정렬·도킹

<img width=500 alt="(판넬)PID직진피드백" src="https://github.com/user-attachments/assets/1d3c6453-46de-4714-b307-d78ee63c1b52" />

- 정밀 정차·도킹을 위해 ArUco 마커 오차를 PID 제어로 보정
- 카메라 프레임에서 ArUco 마커를 검출해 좌우·거리 오차를 계산
- 오차를 PID로 제어해 속도 명령(`/cmd_vel`) 생성 → 목표 거리·각도 도달 시 정지

<!-- - 역할 분리를 위해 로컬 제어로 동작시켜 Nav2 경로 제어와 분리시킴 -->


<br/>
<br/>
<br/>

### 7. [Kalman Filter] IMU 노이즈 보정

<img width=300 alt="구현설명5-kalman-filter" src="https://github.com/user-attachments/assets/074bed43-e96a-474c-bde6-2d4cc12edc12" />

- IMU 노이즈 보정을 위한 칼만 필터 데이터는 수집 완료
- 적용 목적은 IMU 흔들림 기반 충돌 로그 기록이었으나 시간 문제로 필터링 파이프라인 및 기능은 적용하지 못함


<br/>
<br/>
<br/>

### 8. [Micro-ROS + Platform.io + Docker] MCU 센서 통신

Rack과 Cart에서 집품을 확인하는 기능에 ESP32 MCU 펌웨어가 사용되었습니다. \
보드의 설정은 Platform.io로 관리되며, 시리얼 통신으로 센서 데이터를 얻습니다. \
Micro-ROS agent로 ROS2 네트워크와 통신합니다. 이는 Docker로 배포하여 환경 설정을 간소화했습니다.

<!-- PlatformIO: 임베디드 보드(ESP32 등)용 펌웨어를 빌드/업로드/모니터링까지 통합 관리해주는 개발 환경(IDE/CLI) -->
<!-- Micro-ROS: ROS 2를 MCU(마이크로컨트롤러)에서도 쓰기 위한 경량 통신/실행 스택으로, 에이전트를 통해 ROS 2 네트워크와 브릿지됨 -->
<!-- Docker: 개발/배포 환경을 컨테이너 이미지로 패키징해 동일한 실행 환경을 재현하는 가상화 도구 -->
<!-- PlatformIO vs Docker: PlatformIO는 MCU 펌웨어의 개발·배포(보드 업로드), Docker는 애플리케이션 런타임 환경을 컨테이너로 재현·배포하는 도구로 목적과 레벨이 다름 -->

**Micro-ROS agent 실행 절차(컨테이너)** 

PlatformIO 타깃 보드: `esp32dev`(cart), `esp32-s3-devkitc-1`(rack)

1. [ROScar Docker 이미지](https://hub.docker.com/r/yonmilk/roscars) 내려받기 <br/> 
    ```
    docker pull yonmilk/roscars:rack-esp32-s3-wroom-1-0.1.0
    ```
2. MCU 연결 및 디바이스 확인(호스트) <br/>
     USB 연결 후 `/dev/ttyACM*` 또는 `/dev/ttyUSB*` 확인
3. 컨테이너 실행 + 시리얼 디바이스 매핑 <br/>
    ```
    docker run --rm -it --device /dev/ttyACM0 --group-add dialout yonmilk/roscars:rack-esp32-s3-wroom-1-0.1.0
    ```
4. 컨테이너 내부에서 Micro-ROS agent 실행 <br/>
    ```
    ros2 run micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4
    ```
5. 연결 확인 <br/>
  Agent 로그에 XRCE-DDS 세션/클라이언트 연결 로그가 출력되는지 확인



<br/>
<br/>
<br/>
<br/>
<br/>



## GUI

<img width=500 alt="System_Flow" src="https://github.com/user-attachments/assets/4c92156c-8b79-4134-933d-beec5a7c9055" />

<table>
  <tr>
    <td>
      <img
        src="https://github.com/user-attachments/assets/595427b8-d886-4ed8-9999-19cb6dd8d048"
        alt="G1 상품QR코드스캔"
      />
    </td>
    <td>
      <img
        src="https://github.com/user-attachments/assets/1d075177-cbf8-4cdf-b10e-76b731f1fc72"
        alt="G2 스캔결과"
      />
    </td>
    <td>
      <img
        src="https://github.com/user-attachments/assets/4e8da657-8b77-488d-beb8-c55f10588af2"
        alt="G3 배송요청"
      />
    </td>
  </tr>
</table>


<img alt="gui_manager" src="https://github.com/user-attachments/assets/a9aebdcf-7d43-450d-94ca-191bf8ee6883" />



## 설계

- [Interface Specification](https://github.com/addinedu-ros-8th/ros-repo-3/wiki/Interface-Specification)

<!--
- [User Requirements Specification (URS)]()
- [System Requirements Specification (SRS)]()
- [Scenario]()
- [GUI Specification]()
-->

### 시스템 구조

<table>
  <colgroup>
    <col style="width:33%;">
    <col style="width:66%;">
  </colgroup>
  <tr>
    <td>
      <img
        src="https://github.com/user-attachments/assets/501b885f-74e1-4654-87e8-1e59daebe873"
        alt="시스템_아키텍처(희망)"
        style="width:100%;"
      />
    </td>
    <td>
      <img
        src="https://github.com/user-attachments/assets/94056eec-1e70-478e-83dd-e1b18555eab6"
        alt="시스템_아키텍처(최종_요약)"
        style="width:100%;"
      />
    </td>
  </tr>
</table>

### 지도

<table>
  <colgroup>
    <col style="width:33%;">
    <col style="width:66%;">
  </colgroup>
  <tr>
    <td>
      <img
        src="https://github.com/user-attachments/assets/ae84120e-f33c-49fb-aa64-16e9c6d39134"
        alt="Map_Design(가로)"
        style="width:100%;"
      />
    </td>
    <td>
      <img
        src="https://github.com/user-attachments/assets/6570ac7c-5f10-4f29-a221-83a137359cc1"
        alt="Map_Design(세로)"
        style="width:100%;"
      />
    </td>
  </tr>
</table>


### 시퀀스 다이어그램
<table>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/e60af90f-ce11-4c58-aee3-7001146b20ed"
           alt="User_Device Provisioning and User Authentication" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/2bf98300-5b20-4838-b6d1-3b1b80192b6e"
           alt="inventory_verification_mode" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/b7fc5377-682b-4336-b87f-c37acd78b583"
           alt="task_assignment_flow" />
    </td>
  </tr>

  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/55cf4cb9-81a4-4030-a384-328d97af4f4a"
           alt="scenario-1 Product information inquiry" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/78365892-e2bd-4018-a900-f389ea34d113"
           alt="scenario-2 Product request" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/444b57e5-6e23-4460-80b9-2ce5beed4e7d"
           alt="Roscar_monitoring_and_management" />
    </td>
  </tr>

  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/b311016f-07cd-4b23-9a52-61ca2e0fdfa0"
           alt="log_and_audit_flow" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/ebdb01e8-2cfe-466e-a65f-1c6c4870fc19"
           alt="Roscar_sensor_and_state_reporting" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/ae8d0b6c-eb7a-404a-885b-a72fe1712782"
           alt="ai_perception_pipeline" />
    </td>
  </tr>

  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/7cc97255-8534-4ddb-b1fb-130ec9fb035b"
           alt="roscar_control_and_motor_flow" />
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/8b26f23e-5930-4df2-84fb-691b6b06f214"
           alt="inventory-verification-mode" />
    </td>
  </tr>
</table>

### 주요 ROS 통신 인터페이스

<img width=600 alt="통신(interface)_요약" src="https://github.com/user-attachments/assets/cc570489-3238-403d-bd4f-1070783b2042" />


### 전체 주행 상태

<img width=400 alt="Robot_State_Diagram-Copy of Copy of roscars drawio" src="https://github.com/user-attachments/assets/d1eee833-0e44-42d5-a760-833a7ec55688" />


### 데이터 구조

<table>
  <tr>
    <td>
      <img
        src="https://github.com/user-attachments/assets/a9d2a7fb-de53-4729-9ea2-5cc012d70014"
        alt="roscars_erd"
      />
		<td />
		<td>
      <img
        src="https://github.com/user-attachments/assets/c6ac51fb-b600-438e-ae93-5370eff47f5d"
        alt="roscars_erd"
      />
    </td>
  </tr>
</table>

### Hardware 구성

<img alt="pinky-spec-3" src="https://github.com/user-attachments/assets/b610a600-ee33-43a0-8099-66eb5bc542e7" />


