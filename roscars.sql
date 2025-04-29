CREATE TABLE `RosCars` (
  `roscar_id` integer PRIMARY KEY,
  `roscar_name` varchar(255),
  `battery_percentage` integer,
  `operational_status` ENUM ('STANDBY', 'DRIVING', 'CHARGING', 'ERROR', 'EMERGENCY_STOP'),
  `roscar_ip_v4` text
);

CREATE TABLE `RosCarDrivingStatus` (
  `driving_status_id` integer PRIMARY KEY,
  `roscar_id` integer,
  `status_type` ENUM ('PICKUP', 'DELIVERY', 'RETURN', 'GO_TO_STANDBY_ZONE', 'GO_TO_CHARGING_ZONE', 'DISCONNECT'),
  `is_enabled` boolean
);

CREATE TABLE `Delivery` (
  `delivery_id` integer PRIMARY KEY,
  `roscar_id` integer,
  `user_id` integer,
  `driving_status_id` integer,
  `delivery_status` ENUM ('TO_DO', 'IN_PROGRESS', 'COMPLETING'),
  `delivery_start_time` timestamp,
  `delivery_end_time` timestamp
);

CREATE TABLE `Task` (
  `task_id` integer PRIMARY KEY,
  `delivery_id` integer,
  `shoes_id` integer,
  `task_status` ENUM ('TO_DO', 'IN_PROGRESS', 'DONE'),
  `task_start_time` timestamp,
  `task_end_time` timestamp
);

CREATE TABLE `ShoesModel` (
  `shoes_model_id` integer PRIMARY KEY,
  `name` text,
  `size` integer,
  `color_name` ENUM ('black', 'white', 'snow', 'red', 'tomato', 'salmon', 'coral', 'pink', 'magenta', 'hotpink', 'raspberry', 'orange', 'chocolate', 'yellow', 'khaki', 'gold', 'green', 'greenyellow', 'olive', 'olivedrab', 'azure', 'skyblue', 'aqua', 'aquamarine', 'cyan', 'blue', 'navy', 'violet', 'lavender', 'indigo', 'purple')
);

CREATE TABLE `Location` (
  `location_id` integer PRIMARY KEY,
  `name` text,
  `floor_level` integer,
  `zone_number` integer,
  `map_x` float,
  `map_y` float,
  `aruco_id` integer,
  `updated_at` timestamp
);

CREATE TABLE `Inventory` (
  `inventory_id` integer PRIMARY KEY,
  `location_id` integer,
  `shoes_model_id` integer,
  `quantity` integer,
  `last_updated` timestamp
);

CREATE TABLE `QRCode` (
  `qrcode_id` integer PRIMARY KEY,
  `inventory_id` integer,
  `qrcode_data` varchar(255)
);

CREATE TABLE `User` (
  `user_id` integer PRIMARY KEY,
  `user_name` varchar(255),
  `user_role` ENUM ('ADMIN', 'WORKER'),
  `can_call_roscar` boolean,
  `password` varchar(255)
);

CREATE TABLE `DeliveryEventLog` (
  `event_id` integer PRIMARY KEY,
  `delivery_id` integer,
  `previous_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `new_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `changed_at` timestamp,
  `changed_by_user_id` integer
);

CREATE TABLE `TaskEventLog` (
  `event_id` integer PRIMARY KEY,
  `task_id` integer,
  `previous_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `new_event` ENUM ('WAIT', 'PROGRESS_START', 'COMPLET', 'CANCEL', 'FAILE'),
  `changed_at` timestamp
);

CREATE TABLE `RosCarEventLog` (
  `event_id` integer PRIMARY KEY,
  `roscar_id` integer,
  `task_id` integer,
  `event_type` ENUM ('PICKUP_START', 'EMERGENCY_STOP', 'COLLISION_AVOID', 'ERROR', 'CHARGING_START', 'CHARGING_COMPLET'),
  `event_timestamp` timestamp
);

CREATE TABLE `InventoryEventLog` (
  `event_id` integer PRIMARY KEY,
  `actor_user_id` integer,
  `item_id` integer,
  `roscar_id` integer,
  `quantity` integer,
  `event_timestamp` timestamp
);

ALTER TABLE `RosCarDrivingStatus` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`user_id`) REFERENCES `User` (`user_id`);

ALTER TABLE `Delivery` ADD FOREIGN KEY (`driving_status_id`) REFERENCES `RosCarDrivingStatus` (`status_type`);

ALTER TABLE `Task` ADD FOREIGN KEY (`delivery_id`) REFERENCES `Delivery` (`delivery_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`shoes_id`) REFERENCES `ShoesModel` (`shoes_model_id`);

ALTER TABLE `Inventory` ADD FOREIGN KEY (`location_id`) REFERENCES `Location` (`location_id`);

ALTER TABLE `Inventory` ADD FOREIGN KEY (`shoes_model_id`) REFERENCES `ShoesModel` (`shoes_model_id`);

ALTER TABLE `QRCode` ADD FOREIGN KEY (`inventory_id`) REFERENCES `Inventory` (`inventory_id`);

ALTER TABLE `DeliveryEventLog` ADD FOREIGN KEY (`delivery_id`) REFERENCES `Delivery` (`delivery_id`);

ALTER TABLE `DeliveryEventLog` ADD FOREIGN KEY (`changed_by_user_id`) REFERENCES `User` (`user_id`);

ALTER TABLE `TaskEventLog` ADD FOREIGN KEY (`task_id`) REFERENCES `Task` (`task_id`);

ALTER TABLE `RosCarEventLog` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);

ALTER TABLE `RosCarEventLog` ADD FOREIGN KEY (`task_id`) REFERENCES `Task` (`task_id`);

ALTER TABLE `InventoryEventLog` ADD FOREIGN KEY (`actor_user_id`) REFERENCES `User` (`user_id`);

ALTER TABLE `InventoryEventLog` ADD FOREIGN KEY (`item_id`) REFERENCES `Inventory` (`inventory_id`);

ALTER TABLE `InventoryEventLog` ADD FOREIGN KEY (`roscar_id`) REFERENCES `RosCars` (`roscar_id`);
