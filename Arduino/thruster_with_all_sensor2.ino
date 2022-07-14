#include <Servo.h>
#include "ping1d.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include "MS5837.h"

long timer = 0;

MPU6050 mpu6050(Wire);
MS5837 sensor;

std_msgs::String pub_msg;
std_msgs::Float32 depth_msg;
std_msgs::Float64 accx_msg;
std_msgs::Float64 accy_msg;
std_msgs::Float64 accz_msg;
std_msgs::Float64 gyrox_msg;
std_msgs::Float64 gyroy_msg;
std_msgs::Float64 gyroz_msg;
std_msgs::Float64 acc_angle_x_msg;
std_msgs::Float64 acc_angle_y_msg;
std_msgs::Float64 gyro_angle_x_msg;
std_msgs::Float64 gyro_angle_y_msg;
std_msgs::Float64 gyro_angle_z_msg;
std_msgs::Float64 angle_x_msg;
std_msgs::Float64 angle_y_msg;
std_msgs::Float64 angle_z_msg;

std_msgs::Float64 distance_r_msg;
std_msgs::Float64 distance_l_msg;
std_msgs::Float64 confidence_r_msg;
std_msgs::Float64 confidence_l_msg;
// std_msgs::String imu_msg;
// std_msgs::String pressure_msg;




Servo servoFrontRight;
Servo servoFrontLeft;
Servo servoMiddleRight;
Servo servoMiddleLeft;
Servo servoBackLeft;
Servo servoBackRight;

// ===== for Arduino Mega ===========
byte pin_fr = 7;
byte pin_fl = 5;
byte pin_mr = 4;
byte pin_ml = 6;
byte pin_br = 3;
byte pin_bl = 2;*/
byte arduinoRxPin_R = 19; //Serial1 rx
byte arduinoTxPin_R = 18; //Serial1 tx
byte arduinoRxPin_B = 17; //Serial1 rx
byte arduinoTxPin_B = 16; //Serial1 tx

//SoftwareSerial Serial_Right = SoftwareSerial(arduinoRxPin_R, arduinoTxPin_R);
//SoftwareSerial Serial_Back = SoftwareSerial(arduinoRxPin_B, arduinoTxPin_B);

ros::NodeHandle nh;
static Ping1D ping_R { Serial1 };
static Ping1D ping_B { Serial2 };

int pwm_fr=1500 ,pwm_fl=1500,pwm_mr=1500,pwm_ml=1500,pwm_br=1500,pwm_bl=1500;
String pwm_value="1500 1500 1500 1500 1500 1500";
double AccX,AccY,AccZ,GyroX,GyroY,GyroZ,AccAngleX,AccAngleY,GyroAngleX,GyroAngleY,GyroAngleZ,AngleX,AngleY,AngleZ;
double distance_Right,confidence_Right,distance_Left,confidence_Left;
// String imu_value="imu value";
// String pressure_value="pressure value";
char pwm[30]= "1500 1500 1500 1500 1500 1500";
// char imu_c[9]= "imu value";
// char pressure_c[14]= "pressure value";


void message7(const std_msgs::String& msg)
{
  //Serial.print("Inside Message7 ");
  //Serial.print(int(msg.data));
  pwm_value= msg.data;

  
  
  pwm_fr= (pwm_value.substring(0,4)).toInt();
  pwm_fl= (pwm_value.substring(5,9)).toInt();
  pwm_mr= (pwm_value.substring(10,14)).toInt();
  pwm_ml= (pwm_value.substring(15,19)).toInt();
  pwm_br= (pwm_value.substring(20,24)).toInt();
  pwm_bl= (pwm_value.substring(25)).toInt();
  
}

void message_middle(const std_msgs::String& msg)
{
  //Serial.print("Inside Message Middle ");
  //Serial.print(int(msg.data));
  pwm_value= msg.data;

  pwm_mr= (pwm_value.substring(0,4)).toInt();
  pwm_ml= (pwm_value.substring(5,9)).toInt();  
}

void message_car(const std_msgs::String& msg)
{
  //Serial.print("Inside Message Car ");
  //Serial.print(int(msg.data));
  pwm_value= msg.data;

  pwm_fr= (pwm_value.substring(0,4)).toInt();
  pwm_fl= (pwm_value.substring(5,9)).toInt();  
  pwm_br= (pwm_value.substring(10,14)).toInt();
  pwm_bl= (pwm_value.substring(15,19)).toInt();  
}


ros::Publisher pub_accx("accx", &accx_msg);
ros::Publisher pub_accy("accy", &accy_msg);
ros::Publisher pub_accz("accz", &accz_msg);
ros::Publisher pub_gyrox("gyrox", &gyrox_msg);
ros::Publisher pub_gyroy("gyroy", &gyroy_msg);
ros::Publisher pub_gyroz("gyroz", &gyroz_msg);
ros::Publisher pub_acc_angle_x("acc_angle_x", &acc_angle_x_msg);
ros::Publisher pub_acc_angle_y("acc_angle_y", &acc_angle_y_msg);
ros::Publisher pub_gyro_angle_x("gyro_angle_x", &gyro_angle_x_msg);
ros::Publisher pub_gyro_angle_y("gyro_angle_y", &gyro_angle_y_msg);
ros::Publisher pub_gyro_angle_z("gyro_angle_z", &gyro_angle_z_msg);
ros::Publisher pub_angle_x("angle_x", &angle_x_msg);
ros::Publisher pub_angle_y("angle_y", &angle_y_msg);
ros::Publisher pub_angle_z("angle_z", &angle_z_msg);

ros::Publisher pub_dist_r("dist_r", &distance_r_msg);
ros::Publisher pub_dist_l("dist_l", &distance_l_msg);
ros::Publisher pub_con_r("con_r", &confidence_r_msg);
ros::Publisher pub_con_l("con_l", &confidence_l_msg);

ros::Publisher pub("Verify", &pub_msg);
ros::Publisher pub_depth("Depth", &depth_msg);
ros::Subscriber<std_msgs::String> sub("PWM_VALUE", &message7);
ros::Subscriber<std_msgs::String> sub_only_depth("PWM_VALUE_Middle", &message_middle);
ros::Subscriber<std_msgs::String> sub_only_car("PWM_VALUE_car", &message_car);

void setup()
{

  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_only_depth);
  nh.subscribe(sub_only_car);
  nh.advertise(pub);
  nh.advertise(pub_depth);
  
  nh.advertise(pub_accx);
  nh.advertise(pub_accy);
  nh.advertise(pub_accz);
  nh.advertise(pub_gyrox);
  nh.advertise(pub_gyroy);
  nh.advertise(pub_gyroz);
  nh.advertise(pub_acc_angle_x);
  nh.advertise(pub_acc_angle_y);
  nh.advertise(pub_gyro_angle_x);
  nh.advertise(pub_gyro_angle_y);
  nh.advertise(pub_gyro_angle_z);
  nh.advertise(pub_angle_x);
  nh.advertise(pub_angle_y);
  nh.advertise(pub_angle_z);

  nh.advertise(pub_dist_r);
  nh.advertise(pub_dist_l);
  nh.advertise(pub_con_r);
  nh.advertise(pub_con_l);
  // nh.advertise(pub_imu);
  // nh.advertise(pub_pressure);
  
  servoFrontRight.attach(pin_fr);
  servoFrontLeft.attach(pin_fl);
  servoMiddleRight.attach(pin_mr);
  servoMiddleLeft.attach(pin_ml);
  servoBackRight.attach(pin_br);
  servoBackLeft.attach(pin_bl);
  
  servoFrontRight.writeMicroseconds(1500);
  servoFrontLeft.writeMicroseconds(1500);
  servoMiddleRight.writeMicroseconds(1500);
  servoMiddleLeft.writeMicroseconds(1500);
  servoBackRight.writeMicroseconds(1500);
  servoBackLeft.writeMicroseconds(1500);
 
  delay(7000); // delay to allow the ESC to recognize the stopped signal<br />

  Wire.begin();
  Serial_Right.begin(9600);
  Serial_Back.begin(9600);
  Serial.begin(115200);   //ek bar dekh lena sonar 115200 braud pe chal raha h and imu 9600 braud rate pe chalta h

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //initialize sonar right
  while (!ping_R.initialize()) {
    
    delay(2000);
  }
  while (!ping_B.initialize()) {
    
    delay(2000);
  }


  

  
  while (!sensor.init()) {
  
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  
}

void MOVE()
{
  //servoFrontRight.write(pwm_fr);


  
  servoFrontRight.writeMicroseconds(pwm_fr);
  servoFrontLeft.writeMicroseconds(pwm_fl);
  servoMiddleRight.writeMicroseconds(pwm_mr);
  servoMiddleLeft.writeMicroseconds(pwm_ml);
  servoBackRight.writeMicroseconds(pwm_br);
  servoBackLeft.writeMicroseconds(pwm_bl);

    delay(10);
    pwm_value.toCharArray(pwm, 30);
    pub_msg.data = pwm;
    pub.publish(&pub_msg);
}


void ACCX(){
  mpu6050.update();
  AccX=mpu6050.getAccX();
  accx_msg.data=AccX;
  pub_accx.publish(&accx_msg);
  delay(10);
}
void ACCY(){
  mpu6050.update();
  AccY=mpu6050.getAccY();
  accy_msg.data=AccY;
  pub_accy.publish(&accy_msg);
  delay(10);
}
void ACCZ(){
  mpu6050.update();
  AccZ=mpu6050.getAccZ();
  accz_msg.data=AccZ;
  pub_accz.publish(&accz_msg);
  delay(10);
}
void GYROX(){
  mpu6050.update();
  GyroX=mpu6050.getGyroX();
  gyrox_msg.data=GyroX;
  pub_gyrox.publish(&gyrox_msg);
  delay(10);
}
void GYROY(){
  mpu6050.update();
  GyroY=mpu6050.getGyroY();
  gyroy_msg.data=GyroY;
  pub_gyroy.publish(&gyroy_msg);
  delay(10);
}
void GYROZ(){
  mpu6050.update();
  GyroZ=mpu6050.getGyroZ();
  gyroz_msg.data=GyroZ;
  pub_gyroz.publish(&gyroz_msg);
  delay(10);
}
void ACC_ANGLE_X(){
  mpu6050.update();
  AccAngleX=mpu6050.getAccAngleX();
  acc_angle_x_msg.data=AccAngleX;
  pub_acc_angle_x.publish(&acc_angle_x_msg);
  delay(10);
}
void ACC_ANGLE_Y(){
  mpu6050.update();
  AccAngleY=mpu6050.getAccAngleY();
  acc_angle_y_msg.data=AccAngleY;
  pub_acc_angle_y.publish(&acc_angle_y_msg);
  delay(10);
}
void GYRO_ANGLE_X(){
  mpu6050.update();
  GyroAngleX=mpu6050.getGyroAngleX();
  gyro_angle_x_msg.data=GyroAngleX;
  pub_gyro_angle_x.publish(&gyro_angle_x_msg);
  delay(10);
}
void GYRO_ANGLE_Y(){
  mpu6050.update();
  GyroAngleY=mpu6050.getGyroAngleY();
  gyro_angle_y_msg.data=GyroAngleY;
  pub_gyro_angle_y.publish(&gyro_angle_y_msg);
  delay(10);
}
void GYRO_ANGLE_Z(){
  mpu6050.update();
  GyroAngleZ=mpu6050.getGyroAngleZ();
  gyro_angle_z_msg.data=GyroAngleZ;
  pub_gyro_angle_z.publish(&gyro_angle_z_msg);
  delay(10);
}
void ANGLE_X(){
  mpu6050.update();
  AngleX=mpu6050.getAngleX();
  angle_x_msg.data=AngleX;
  pub_angle_x.publish(&angle_x_msg);
  delay(10);
}
void ANGLE_Y(){
  mpu6050.update();
  AngleY=mpu6050.getAngleY();
  angle_y_msg.data=AngleY;
  pub_angle_y.publish(&angle_y_msg);
  delay(10);
}
void ANGLE_Z(){
  mpu6050.update();
  AngleZ=mpu6050.getAngleZ();
  angle_z_msg.data=AngleZ;
  pub_angle_z.publish(&angle_z_msg);
  delay(10);
}

void DIST_R(){
  while (!myPing.update()) {
        Serial.println("Ping device update failed");
    }
    distance_Right = ping_R.distance();
    distance_r_msg.data=distance_Right;
    pub_dist_r.publish(&distance_r_msg);
    delay(10);
    
}

void DIST_L(){
  while (!myPing.update()) {
        Serial.println("Ping device update failed");
    }
    distance_Left = ping_L.distance();
    distance_l_msg.data=distance_Left;
    pub_dist_l.publish(&distance_l_msg);
    delay(10);
    
}

void CON_L(){
  while (!myPing.update()) {
        Serial.println("Ping device update failed");
    }
    confidence_Left = ping_L.confidence();
    confidence_l_msg.data=confidence_Left;
    pub_con_l.publish(&confidence_l_msg);
    delay(10);
    
}

void CON_R(){
  while (!myPing.update()) {
        Serial.println("Ping device update failed");
    }
    confidence_Right = ping_R.confidence();
    confidence_r_msg.data=confidence_Right;
    pub_con_r.publish(&confidence_r_msg);
    delay(10);
    
}
/*void IMU_DATA(){

      mpu6050.update();
      
//    imu_value.toCharArray(imu_c, 10);
//    imu_msg.data = imu_c;
//    pub_imu.publish(&imu_msg);

      float accX=mpu6050.getAccX();
      float accY=mpu6050.getAccY();
      float accZ=mpu6050.getAccZ();

      float gyroX=mpu6050.getGyroX();
      float gyroY=mpu6050.getGyroY();
      float gyroZ=mpu6050.getGyroZ();

      float accAngleX=mpu6050.getAccAngleX();
      float accAngleY=mpu6050.getAccAngleY();

      float gyroAngleX=mpu6050.getGyroAngleX();
      float gyroAngleY=mpu6050.getGyroAngleY();
      float gyroAngleZ=mpu6050.getGyroAngleZ();

      float angleX=mpu6050.getAngleX();
      float angleY=mpu6050.getAngleY();
      float angleZ=mpu6050.getAngleZ();

}*/

void PRESSURE_DATA(){

  sensor.read();

  float pressure= sensor.pressure();
  float depth = sensor.depth();
 

   depth_msg.data = depth;
   pub_depth.publish(&depth_msg);
   delay(10);

//    pressure_value.toCharArray(pressure_c, 15);
//    pressure_msg.data = pressure_c;
//    pub_pressure.publish(&pressure_msg);

}

/*void SONAR_DATA(){
    //Right sonar values
    float distance_Right = ping_R.distance();
    float confidence_Right = ping_R.confidence();

    //Back sonar values
    float distance_Back = ping_B.distance();
    float confidence_Back = ping_B.confidence();
  }*/


 
void loop()
{
    Serial.println("Inside loop");
    
     MOVE();

    // IMU_DATA();
    PRESSURE_DATA();
    ACCX();
    ACCY();
    ACCZ();
    GYROX();
    GYROY();
    GYROZ();
    ACC_ANGLE_X();
    ACC_ANGLE_Y();
    GYRO_ANGLE_X();
    GYRO_ANGLE_Y();
    GYRO_ANGLE_Z();
    ANGLE_X();
    ANGLE_Y();
    ANGLE_Z();

    PRESSURE_DATA();


    //SONAR_DATA();
    DIST_R();
    DIST_L();
    CON_R();
    CON_L();


    


   nh.spinOnce();

}
