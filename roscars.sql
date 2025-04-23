CREATE TABLE `Robot` (
  `robot_id` integer PRIMARY KEY,
  `robot_name` varchar(255),
  `battery_level` integer,
  `current_status` varchar(255)
);

CREATE TABLE `Functionality` (
  `function_id` integer PRIMARY KEY,
  `function_name` varchar(255),
  `priority` varchar(255)
);

CREATE TABLE `RobotFunction` (
  `robot_function_id` integer PRIMARY KEY,
  `robot_id` integer,
  `function_id` integer,
  `is_available` boolean
);

CREATE TABLE `Task` (
  `task_id` integer PRIMARY KEY,
  `robot_id` integer,
  `function_id` integer,
  `task_name` varchar(255),
  `status` varchar(255),
  `priority_level` varchar(255),
  `start_time` timestamp,
  `end_time` timestamp
);

CREATE TABLE `Log` (
  `log_id` integer PRIMARY KEY,
  `robot_id` integer,
  `task_id` integer,
  `timestamp` timestamp,
  `event_type` varchar(255),
  `description` text
);

CREATE TABLE `Inventory` (
  `item_id` integer PRIMARY KEY,
  `location` varchar(255),
  `aruco_id` integer,
  `quantity` integer,
  `last_updated` timestamp
);

CREATE TABLE `InventoryChangeLog` (
  `change_id` integer PRIMARY KEY,
  `item_id` integer,
  `robot_id` integer,
  `change_type` varchar(255),
  `change_by` varchar(255),
  `timestamp` timestamp
);

CREATE TABLE `User` (
  `user_id` integer PRIMARY KEY,
  `name` varchar(255),
  `role` varchar(255),
  `call_robot` boolean
);

ALTER TABLE `RobotFunction` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robot` (`robot_id`);

ALTER TABLE `RobotFunction` ADD FOREIGN KEY (`function_id`) REFERENCES `Functionality` (`function_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robot` (`robot_id`);

ALTER TABLE `Task` ADD FOREIGN KEY (`function_id`) REFERENCES `Functionality` (`function_id`);

ALTER TABLE `Log` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robot` (`robot_id`);

ALTER TABLE `Log` ADD FOREIGN KEY (`task_id`) REFERENCES `Task` (`task_id`);

ALTER TABLE `InventoryChangeLog` ADD FOREIGN KEY (`item_id`) REFERENCES `Inventory` (`item_id`);

ALTER TABLE `InventoryChangeLog` ADD FOREIGN KEY (`robot_id`) REFERENCES `Robot` (`robot_id`);
