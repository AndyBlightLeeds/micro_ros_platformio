#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#define SSID "YOUR_SSID"
#define SSID_PW "your_password"

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

static const char *TAG = "main";

static const char *k_twist = "cmd_vel";
static rcl_subscription_t subscriber_twist;
static geometry_msgs__msg__Twist *twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

#define RCCHECK(fn)                                                    \
  {                                                                    \
    rcl_ret_t temp_rc = fn;                                            \
    if ((temp_rc != RCL_RET_OK)) {                                     \
      printf("Failed status on line %d: %d. Message: %s, Aborting.\n", \
             __LINE__, (int)temp_rc, rcl_get_error_string().str);      \
      rcutils_reset_error();                                           \
      error_loop(temp_rc);                                             \
    }                                                                  \
  }

#define RCSOFTCHECK(fn)                                               \
  {                                                                   \
    rcl_ret_t temp_rc = fn;                                           \
    if ((temp_rc != RCL_RET_OK)) {                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, \
             (int)temp_rc);                                           \
    }                                                                 \
  }


// Error handle loop
void error_loop(rcl_ret_t rc) {
  ESP_LOGE(TAG, "Entering Error Loop with rc = %d", rc);
  while (1) {
    delay(100);
  }
}

void twist_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msg_in;
  ESP_LOGI(TAG, "Received: linear.x %f, angular.z %f", msg->linear.x,
           msg->angular.z);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);

  char ssid[] = SSID;
  char ssid_pw[] = SSID_PW;
  IPAddress agent_ip(192, 168, 54, 2);
  const uint16_t k_agent_port = 8888;
  set_microros_wifi_transports(ssid, ssid_pw, agent_ip, k_agent_port);
  delay(2000);
  Serial.println("Connected");

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_platformio_node_publisher"));

  // create timer,
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber_twist, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), k_twist));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_twist, &twist_msg, &twist_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
