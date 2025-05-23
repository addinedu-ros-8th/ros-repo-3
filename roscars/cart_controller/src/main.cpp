#include <Arduino.h>
#include <esp_system.h>               // esp_get_free_heap_size() 사용 시 필요
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>     // micro-ROS for PlatformIO

// ─── 사용자 설정 ───────────────────────────────────
#define FSR_PIN            4
#define VCC                3.3f
#define THRESHOLD_VOLTAGE  0.3f      // 문턱 전압

// ─── micro-ROS 객체 ─────────────────────────────────
rcl_publisher_t           publisher;
std_msgs__msg__Int32      msg;
rclc_executor_t           executor;
rclc_support_t            support;
rcl_allocator_t           allocator;
rcl_node_t                node;
rcl_timer_t               timer;

// ─── 타이머 콜백 함수 ────────────────────────────────
void timer_callback(rcl_timer_t* timer, int64_t /*last_call_time*/) {
  int raw = analogRead(FSR_PIN);
  float voltage = raw * VCC / 4095.0f;
  msg.data = (voltage > THRESHOLD_VOLTAGE) ? 1 : 0;
  rcl_publish(&publisher, &msg, nullptr);
  Serial.print("FSR signal: ");
  Serial.println(msg.data);
}

// ─── setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000);

  // Free heap 정보 출력
  Serial.print("Free heap at start: ");
  Serial.println(esp_get_free_heap_size());

  // micro-ROS 시리얼 전송 초기화
  set_microros_serial_transports(Serial);
  delay(2000); // 에이전트 안정화 대기

  // rcl / rclc 초기화
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 25);
  rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator);

  // 노드 및 퍼블리셔 생성
  rclc_node_init_default(&node, "fsr_serial_node", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/cart_is_on"
  );

  // 타이머(100ms) 생성 및 executor에 추가
  const uint64_t timer_period_ns = RCL_MS_TO_NS(100);
  rclc_timer_init_default(&timer, &support, timer_period_ns, timer_callback);
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  Serial.print("Free heap after setup: ");
  Serial.println(esp_get_free_heap_size());
}

// ─── loop ────────────────────────────────────────────
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
