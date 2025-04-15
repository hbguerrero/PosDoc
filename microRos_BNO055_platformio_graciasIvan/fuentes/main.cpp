#include <micro_ros_platformio.h>
#include <Arduino.h>

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

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;
rclc_executor_t executor_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long lastTime = 0;
float ImuIni;

#define LED_PIN 2

#define L1_PIN 32
#define R1_PIN 33

#define L2_PIN 26
#define R2_PIN 27


// subscriber
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg_led;
rclc_executor_t executor_sub;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //msg.data++;

    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);

    delay(100);

    msg.data = int(float(event.orientation.x));
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg_led = (const std_msgs__msg__Float32 *)msgin;
  // (condition) ? (true exec):(false exec)
  //digitalWrite(LED_PIN, (msg_led->data == 0) ? LOW : HIGH);

  //digitalWrite(L_PIN, (msg_led->data == 0) ? LOW : HIGH); 

  if ((msg_led -> data < 2) and (msg_led -> data > -2) ) {
    analogWrite(L1_PIN, 255);
    analogWrite(R1_PIN,0);
    analogWrite(L2_PIN, 255);
    analogWrite(R2_PIN, 0);
  } 
  else if ((msg_led -> data < -2)) {
    analogWrite(L1_PIN, (255*0.5));
    analogWrite(R1_PIN, 0);
    analogWrite(L2_PIN, 255);
    analogWrite(R2_PIN, 0);
  }  

  else {
    analogWrite(L1_PIN, 255);
    analogWrite(R1_PIN, 0);
    analogWrite(L2_PIN, (255*0.5));
    analogWrite(R2_PIN, 0);
  }  

  // Process message
  //printf("Received: %d\n", msg_led->data);
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(L1_PIN, OUTPUT);
  pinMode(R1_PIN, OUTPUT);
  pinMode(L2_PIN, OUTPUT);
  pinMode(R2_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  analogWrite(L1_PIN, 0);
  analogWrite(R1_PIN, 0);  
  analogWrite(L2_PIN, 0);
  analogWrite(R2_PIN, 0);  
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

  //delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  // const char topic_name_led[] = "xiao_led_state";
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "u"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "bno"));

  // create timer,
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
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_led, &subscription_callback, ON_NEW_DATA))
  //msg.data = 0;
}

void loop() {
  //delay(100);
  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
