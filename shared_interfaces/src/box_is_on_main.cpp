#include <Arduino.h>
#include <esp_system.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>
#include "config.h"

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

char topic_name[64];

void timer_callback(rcl_timer_t* timer, int64_t /*last_call_time*/) {
  int raw = analogRead(FSR_PIN);
  float voltage = raw * VCC / 4095.0f;
  msg.data = (voltage > THRESHOLD_VOLTAGE) ? 1 : 0;
  rcl_publish(&publisher, &msg, nullptr);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  snprintf(topic_name, sizeof(topic_name), "/%s/is_on", DEVICE_NAMESPACE);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, nullptr, &allocator);

  rclc_node_init_default(&node, NODE_NAME, "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    topic_name
  );

  const uint64_t timer_period_ns = RCL_MS_TO_NS(100);
  rclc_timer_init_default(&timer, &support, timer_period_ns, timer_callback);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
