#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg_heartbeat;
std_msgs__msg__Float32 msg_heartbeat;
std_msgs__msg__Float32 msg_heartbeatIni;
rclc_executor_t executor_pub;
rcl_timer_t timer;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long lastTime = 0;
float ImuIni;

// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_led;
rclc_executor_t executor_sub;

#define LED_PIN 2
#define L_PIN 32
#define R_PIN 33

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

/**
 * @brief loop to indicate error with blinking LED
 *
 */
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_heartbeat, NULL));
    //msg_heartbeat.data++;

    /* Get a new sensor event */ 
     sensors_event_t event; 
     bno.getEvent(&event);

    /*
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
    */
  

    delay(100);
  
    //msg_heartbeat.data = int(float(myEulerData.h) / 16.00) - ImuIni;       //Convert to degree
    msg_heartbeat.data = int(float(event.orientation.x));       //Convert to degrees
    //msg_heartbeat.data = ImuIni;
    
  }
  
}

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int32 *msg_led = (const std_msgs__msg__Int32 *)msgin;
  // (condition) ? (true exec):(false exec)
  digitalWrite(LED_PIN, (msg_led->data == 0) ? LOW : HIGH);

   // Process message
  printf("Received: %d\n", msg_led->data);
}

void setup()
{
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  //delay(2000);

  bno.setExtCrystalUse(true);


  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_xiao_node", "", &support));

  // create subscriber
  // const char topic_name_led[] = "xiao_led_state";
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "drone_node"));

  // create publisher
  // const char topic_name_heatbeat[] = "xiao_heartbeat";
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "xiao_heartbeat"));

  // create timer, called every 1000 ms to publish heartbeat
  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_led, &subscription_callback, ON_NEW_DATA));

  //msg_heartbeat.data = 0.0;       //Convert to degrees
}

void loop()
{
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}