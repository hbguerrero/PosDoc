#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 outputMessage;

//ros::Publisher pub("info_back", &outputMessage);

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int sensorAddress = 0x28 >> 1;
float XAngle=0;
float error=0;

void callBackFunction(const std_msgs::Float32 &inputMessage){
  outputMessage.data = 2*inputMessage.data;
  //pub.publish("info_back", &outputMessage);
}

ros::Subscriber<std_msgs::Float32> sub("information", callBackFunction );

std_msgs::Float32 temp_msg;
ros::Publisher pub("info_back", &temp_msg);

void setup() {
  //Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master) 

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

}

long publisher_timer;

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event;
  bno.getEvent(&event);
  

  if (millis() > publisher_timer) {
  // step 1: request reading from sensor 
    //Wire.requestFrom(sensorAddress,3); 
    delay(10);
    
    //if (Wire.available())  // if bytes were received 
    //{
      
     
      byte msb;
      byte lsb;
      int PositionX;
      
      msb = event.orientation.x;  // receive high byte (full degrees)
      lsb = event.orientation.x;  // receive low byte (fraction degrees) 
      //PositionX = ((msb) << 4);  // MSB
      //PositionX |= (lsb >> 4);   // LSB

      XAngle = (float)event.orientation.x - 180;
      //if (XAngle>0){
      //  error = 3.14 - XAngle*3.1416/180;
      //}else{
      //  error = -3.14 - XAngle*3.1416/180;  
      //}

      temp_msg.data = (float)XAngle*3.1416/180;
      //temp_msg.data = (float)XAngle + 180;
      pub.publish(&temp_msg);
      //Serial.print(F("Orientation: "));
      //Serial.println(XAngle);
      //Serial.println((float)event.orientation.x);
      //Serial.println((int)event.orientation.x);
      
    //}
  
  //publisher_timer = millis();
  }
   
  delay(BNO055_SAMPLERATE_DELAY_MS);
  nh.spinOnce();
}