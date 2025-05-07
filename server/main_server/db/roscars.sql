-- roscars database

CREATE DATABASE IF NOT EXISTS `roscars` DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

USE `roscars`;

CREATE TABLE `User` (
  `user_id` integer PRIMARY KEY AUTO_INCREMENT,
  `user_name` varchar(255),
  `user_role` ENUM ('ADMIN', 'WORKER'),
  `can_call_roscar` boolean,
  `password` varchar(255)
);

CREATE TABLE `ShoesModel` (
  `shoes_model_id` integer PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `size` integer,
  `color_name` ENUM ('black', 'white', 'snow', 'red', 'tomato', 'salmon', 'coral', 'pink', 'magenta', 'hotpink', 'raspberry', 'orange', 'chocolate', 'yellow', 'khaki', 'gold', 'green', 'greenyellow', 'olive', 'olivedrab', 'azure', 'skyblue', 'aqua', 'aquamarine', 'cyan', 'blue', 'navy', 'violet', 'lavender', 'indigo', 'purple')
);

CREATE TABLE `Location` (
  `location_id` integer PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `floor_level` integer,
  `zone_number` integer,
  `map_x` float,
  `map_y` float,
  `aruco_id` integer,
  `updated_at` timestamp,
  `destination` varchar(255)
);

CREATE TABLE `Inventory` (
  `inventory_id` integer PRIMARY KEY AUTO_INCREMENT,
  `location_id` integer,
  `shoes_model_id` integer,
  `quantity` integer,
  `last_updated` timestamp
);

CREATE TABLE `QRCode` (
  `qrcode_id` integer PRIMARY KEY AUTO_INCREMENT,
  `inventory_id` integer,
  `qrcode_data` varchar(255)
);

CREATE TABLE `RosCars` (
  `roscar_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_name` varchar(255),
  `battery_percentage` integer,
  `operational_status` ENUM ('STANDBY', 'DRIVING', 'CHARGING', 'ERROR', 'EMERGENCY_STOP'),
  `roscar_ip_v4` varchar(15)
);

CREATE TABLE `RosCarDrivingStatus` (
  `driving_status_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `status_type` ENUM ('PICKUP', 'DELIVERY', 'RETURN', 'PRECISION_STOP', 'GO_TO_STANDBY_ZONE', 'GO_TO_CHARGING_ZONE', 'DISCONNECT'),
  `is_enabled` boolean
);

CREATE TABLE `Delivery` (
  `delivery_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `user_id` integer,
  `driving_status_id` integer,
  `delivery_status` ENUM ('TO_DO', 'IN_PROGRESS', 'COMPLETING'),
  `delivery_start_time` timestamp,
  `delivery_end_time` timestamp
);

CREATE TABLE `Task` (
  `task_id` integer PRIMARY KEY AUTO_INCREMENT,
  `delivery_id` integer,
  `shoes_id` integer,
  `task_status` ENUM ('TO_DO', 'IN_PROGRESS', 'DONE'),
  `task_start_time` timestamp,
  `task_end_time` timestamp,
  `location_id` integer
);

CREATE TABLE `PendingRosCars` (
  `pending_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_name` varchar(255),
  `roscar_ip_v4` varchar(15),
  `requested_at` timestamp
);

CREATE TABLE `RosCarConnectionStatus` (
  `status_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `is_connected` boolean,
  `last_heartbeat` timestamp
);

ALTER TABLE `Inventory` ADD FOREIGN KEY (`location_id`) REFERENCES `Location` (`location_id`);

ALTER TABLE `Inventory` ADD FOREIGN KEY (`shoes_model_id`) REFERENCES `ShoesModel` (`shoes_model_id`);

ALTER TABLE `QRCode` ADD FOREIGN KEY (`inventory_id`) REFERENCES `Inventory` (`inventory_id`);

ALTER TABLE `RosCarDrivingStatus` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`user_id`) REFERENCES `User` (`user_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`driving_status_id`) REFERENCES `RosCarDrivingStatus` (`driving_status_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`delivery_id`) REFERENCES `Delivery` (`delivery_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`shoes_id`) REFERENCES `ShoesModel` (`shoes_model_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`location_id`) REFERENCES `Location` (`location_id`);

ALTER TABLE `RosCarConnectionStatus` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);


-- roscars_log database

CREATE DATABASE IF NOT EXISTS `roscars_log` DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;

USE `roscars_log`;

CREATE TABLE `DeliveryEventLog` (
  `event_id` integer PRIMARY KEY AUTO_INCREMENT,
  `delivery_id` integer,
  `previous_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `new_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `changed_at` timestamp,
  `changed_by_user_id` integer
);

CREATE TABLE `TaskEventLog` (
  `event_id` integer PRIMARY KEY AUTO_INCREMENT,
  `task_id` integer,
  `previous_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `new_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `changed_at` timestamp
);

CREATE TABLE `RosCarEventLog` (
  `event_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `task_id` integer,
  `event_type` ENUM ('PICKUP_START', 'EMERGENCY_STOP', 'COLLISION_AVOID', 'ERROR', 'CHARGING_START', 'CHARGING_COMPLET'),
  `event_timestamp` timestamp
);

CREATE TABLE `InventoryEventLog` (
  `event_id` integer PRIMARY KEY AUTO_INCREMENT,
  `actor_user_id` integer,
  `item_id` integer,
  `roscar_id` integer,
  `quantity` integer,
  `event_timestamp` timestamp
);

CREATE TABLE `FileSystemLog` (
  `file_log_id` integer PRIMARY KEY AUTO_INCREMENT,
  `image_path` text,
  `updatetime` timestamp
);

CREATE TABLE `PrecisionStopLog` (
  `log_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `task_id` integer,
  `is_success` boolean,
  `deviation_cm` float,
  `timestamp` timestamp
);

CREATE TABLE `RobotTrajectoryLog` (
  `trajectory_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `task_id` integer,
  `timestamp` timestamp,
  `position_x` float,
  `position_y` float,
  `velocity` float,
  `heading_angle` float
);

CREATE TABLE `SensorFusionRawLog` (
  `sensor_log_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `timestamp` timestamp,
  `lidar_raw` json,
  `imu_data` json,
  `ultrasonic_data` json,
  `camera_frame_id` varchar(255)
);

CREATE TABLE `ControlCommandLog` (
  `command_id` integer PRIMARY KEY AUTO_INCREMENT,
  `roscar_id` integer,
  `timestamp` timestamp,
  `linear_velocity` float,
  `angular_velocity` float,
  `control_source` ENUM ('PATH_PLANNER', 'OBSTACLE_AVOIDANCE', 'EMERGENCY_HANDLER', 'MANUAL_OVERRIDE')
);

CREATE TABLE `TrainingSample` (
  `sample_id` integer PRIMARY KEY AUTO_INCREMENT,
  `sensor_log_id` integer,
  `image_path` text,
  `label` json,
  `is_used` boolean,
  `created_at` timestamp
);

CREATE TABLE `InferenceModel` (
  `model_id` integer PRIMARY KEY AUTO_INCREMENT,
  `name` varchar(255),
  `version` varchar(255),
  `description` varchar(255),
  `created_at` timestamp
);


-- Log DB에 View 생성
-- 로봇 이벤트 로그 + 로봇 이름, 상태, IP 조회
CREATE VIEW roscars_log.view_roscar_event_detail AS
SELECT
  e.event_id,
  e.roscar_id,
  r.roscar_name,
  r.operational_status,
  r.roscar_ip_v4,
  e.event_type,
  e.event_timestamp
FROM roscars_log.RosCarEventLog e
JOIN roscars.RosCars r ON e.roscar_id = r.roscar_id;

-- 작업 이벤트 로그 + 작업 위치 및 신발 모델 정보
CREATE VIEW roscars_log.view_task_event_history AS
SELECT
  e.event_id,
  e.task_id,
  t.task_status,
  s.name AS shoes_name,
  l.name AS location_name,
  e.previous_event,
  e.new_event,
  e.changed_at
FROM roscars_log.TaskEventLog e
JOIN roscars.Task t ON e.task_id = t.task_id
JOIN roscars.ShoesModel s ON t.shoes_id = s.shoes_model_id
JOIN roscars.Location l ON t.location_id = l.location_id;

-- 배송 요청, 요청자, 로봇 이름과 함께 이벤트 타입 확인
CREATE VIEW roscars_log.view_delivery_log AS
SELECT
  e.event_id,
  d.delivery_id,
  u.user_name,
  r.roscar_name,
  e.new_event AS event_type,
  e.changed_at
FROM roscars_log.DeliveryEventLog e
JOIN roscars.Delivery d ON e.delivery_id = d.delivery_id
JOIN roscars.User u ON d.user_id = u.user_id
JOIN roscars.RosCars r ON d.roscar_id = r.roscar_id;


-- 정밀 정지 수행 결과 + 로봇 이름, 작업 번호 표시
CREATE VIEW roscars_log.view_precision_stop_result AS
SELECT
  p.log_id,
  p.roscar_id,
  r.roscar_name,
  p.task_id,
  p.is_success,
  p.deviation_cm,
  p.timestamp
FROM roscars_log.PrecisionStopLog p
JOIN roscars.RosCars r ON p.roscar_id = r.roscar_id;

-- 로봇 주행 이력 + 로봇 이름 확인
CREATE VIEW roscars_log.view_roscar_trajectory AS
SELECT
  t.trajectory_id,
  t.roscar_id,
  r.roscar_name,
  t.task_id,
  t.timestamp,
  t.position_x,
  t.position_y,
  t.velocity,
  t.heading_angle
FROM roscars_log.RobotTrajectoryLog t
JOIN roscars.RosCars r ON t.roscar_id = r.roscar_id;

-- 학습용 센서 로그 + 로봇 이름 매핑
CREATE VIEW roscars_log.view_sensor_for_training AS
SELECT
  s.sensor_log_id,
  r.roscar_name,
  s.timestamp,
  s.lidar_raw,
  s.imu_data,
  s.ultrasonic_data,
  s.camera_frame_id
FROM roscars_log.SensorFusionRawLog s
JOIN roscars.RosCars r ON s.roscar_id = r.roscar_id;
