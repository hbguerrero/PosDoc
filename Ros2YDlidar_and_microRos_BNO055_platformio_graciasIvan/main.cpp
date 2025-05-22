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

float ImuIni;

#define LED_PIN 2
const int C1 = 32; // Entrada de la señal A del encoder.
const int C2 = 33; // Entrada de la señal B del encoder.
const int PWM1 = 13;
const int PWM2 = 12;
const int MOTOR0 = 18; //IZQUIERDA
const int MOTOR1 = 19; //DERECHA

volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 10;  // Tiempo de muestreo

double P = 0;
double R = 35020;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
//double R = 22660;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
//double R = 1460;   //Parece que ppr = 11; reducción = 515; precición cuádrupla =>  R = 11*4*515
double d = 0;
double pwm =0;
int pwm0 = 0;

void encoder(void);

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

  if (millis() - lastTime >= sampleTime || lastTime==0)
  {  // Se actualiza cada sampleTime (milisegundos)
      lastTime = millis();
      P = (n*360.0)/R;
   }
  
  // VALORES NEGATIVOS GIRA DERECHA, VALORS POSITIVOS GIRA IZQUIERDA
  if ((msg_led->data <= 45.0) and (msg_led->data >=-45.0)) {    
    if (P > msg_led->data){
      analogWrite(PWM1, 0); //retrocede o derecha
      analogWrite(PWM2, 64);
      delay(40);
      analogWrite(PWM1, 0);
      analogWrite(PWM2, 0);                            
     
      if(P >= -2 && P <= 2) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 64);
      }
      if(P < -2) { 
      analogWrite(MOTOR1, 32);
      analogWrite(MOTOR0, 64);
      }
      if(P > 2) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 32);
      }     
     }
     
    else if (P<=msg_led->data){
      if (P>msg_led->data - 0.7 ){
        analogWrite(PWM1, 0); //para
        analogWrite(PWM2, 0);
      }
      else{        
        analogWrite(PWM1, 64);  //avance o izquierda
        analogWrite(PWM2, 0);
        delay(40);
        analogWrite(PWM1, 0);
        analogWrite(PWM2, 0);        
      }

      if(P >= -2 && P <= 2) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 64);
      }
      if(P < -2) { 
      analogWrite(MOTOR1, 32);
      analogWrite(MOTOR0, 64);
      }
      if(P > 2) { 
      analogWrite(MOTOR1, 64);
      analogWrite(MOTOR0, 32);
      }      
    }
  
  else {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  } 
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_PIN, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(MOTOR0, OUTPUT);
  pinMode(MOTOR1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);  
  
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

void encoder(void)
{

  ant=act;
  
  if(digitalRead(C1)) bitSet(act,1); else bitClear(act,1);            
  if(digitalRead(C2)) bitSet(act,0); else bitClear(act,0);

    if(n < R) {
    if(ant == 2 && act ==0) n++;
    if(ant == 0 && act ==1) n++;
    if(ant == 3 && act ==2) n++;
    if(ant == 1 && act ==3) n++;
  }
  else {n = 0;}

  if(n > -R){
    if(ant == 1 && act ==0) n--;
  if(ant == 3 && act ==1) n--;
  if(ant == 0 && act ==2) n--;
  if(ant == 2 && act ==3) n--;    
  }
  else {n = 0;}      
}
