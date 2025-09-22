#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <VL53L0X_mod.h>
#include "MPU6050_6Axis_MotionApps612.h"

#include "Cdrv8833.h"

#include "WheelOdometry.h"
#include "DataParser.h"

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

WheelOdometry leftWheel(&I2Cone);
WheelOdometry rightWheel(&I2Ctwo);

VL53L0X_mod sensor;

#define WIFI_SSID "ground"
#define WIFI_PASSWORD "12345678"
#define UDP_PORT 12345

//Motor Pins
#define IN1_PIN 32 //27
#define IN2_PIN 27  //32
#define RCHANNEL 0
#define IN3_PIN 26
#define IN4_PIN 25
#define LCHANNEL 1

DataParser dataParser;  // Create an instance of DataParse
const char* remoteIP = "192.168.1.11"; // Add your PC IP address

int DataIndex;
int L_Speed = 20;
int R_Speed = 20;
int Right_speed = 0;
int Left_speed = 0;
String movement;

WiFiUDP udp;

Cdrv8833 LMotor;
Cdrv8833 RMotor;

MPU6050 mpu(0x68, &I2Cone);

#define INTERRUPT_PIN 15 // use pin 15 for MPU interrupt

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
volatile bool mpuInterrupt = false;

float globalYaw = 0.0;
float globalPitch = 0.0;
float globalRoll = 0.0;
String globalOdom = "";
float globalDistance = 0.0;
String finalData = "";

bool headingKeepingEnabled = false;

float targetHeading = 0; // Target heading (in degrees)
float headingError = 0;  // Current heading error
float previousError = 0; // Error in the previous loop
float integral = 0;      // Integral term for PID
float kp = 5.0, ki = 0.00, kd = 0.0; // PID coefficients


void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {

  Serial.begin(115200);
  RMotor.init(IN1_PIN, IN2_PIN, LCHANNEL, true);
  LMotor.init(IN3_PIN, IN4_PIN, RCHANNEL, false);
  //Serial.println("ok");
  //_delay(750);
  I2Cone.begin(18,19, 400000UL);   //Scl,Sda
  I2Ctwo.begin(21,22, 400000UL); //22,21

  connectToWiFi();


    // Initialize MPU6050
  Serial.println("Initializing IMU device...");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Initialize DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

    //--------------
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  //-------------------------
  
  udp.begin(UDP_PORT);
  Serial.println("UDP Listening on port ");

    sensor.setBus(&I2Ctwo);
  sensor.setTimeout(500);
  
  
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    //while (1) {}
  }
  //sensor.startContinuous(200);

}


void loop() {
  ArduinoOTA.handle();
  process_imu();
  odom();
  //heading_control();
  measure_distance();
  data_to_send();
  udpReceiveTask();
}


void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);//need for esp32c3
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void udpReceiveTask() {
  char packetBuffer[255];

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;  // Null-terminate the string
      //Serial.println("Received UDP packet:");
      //Serial.println(packetBuffer);

      String IncomingData = packetBuffer;

      // --Library use --
      dataParser.parseData(IncomingData, ','); // Pass data and delimiter

      L_Speed = (dataParser.getField(1)).toInt();
      R_Speed = (dataParser.getField(2)).toInt();
      kp = (dataParser.getField(3)).toFloat();
      ki = (dataParser.getField(4)).toFloat();
      kd = (dataParser.getField(5)).toFloat();
      
      Left_speed = L_Speed;
      Right_speed = R_Speed;
      
      movement = (dataParser.getField(0));

        if(((dataParser.getField(7)).toInt()) == 1)
      {
        headingKeepingEnabled = true;
      }
      else
      {
        headingKeepingEnabled = false;
      }

      // Execute movement based on the parsed data
      if (movement == "f") {
        forward();
      } else if (movement == "b") {
        backward();
      } else if (movement == "l") {
        left();
      } else if (movement == "r") {
        right();
      } else if (movement == "s") {
        Stop();
      }
    }
  }
  delay(10);  // Small delay to avoid CPU overload
}

void motor_speed(int Right_Speed,int Left_Speed)
{
Left_speed = Left_Speed;
Right_speed = Right_Speed;
//Serial.println(Left_speed);
}
void forward()
{
  LMotor.move(Left_speed);
  RMotor.move(Right_speed);
  //Serial.println(Left_speed);
  targetHeading = globalYaw;
}
void backward()
{ 
  headingKeepingEnabled = false;
  LMotor.move(-Left_speed);
  RMotor.move(-Right_speed);
}
void right()
{  
  headingKeepingEnabled = false;
  LMotor.move(-Left_speed);
  RMotor.move(Right_speed);
}
void left()
{ 
  headingKeepingEnabled = false;
  LMotor.move(Left_speed);
  RMotor.move(-Right_speed);
}
void Stop()
{
  headingKeepingEnabled = false;
  LMotor.move(0);
  RMotor.move(0);
}


void odom()
{

   // Update odometry for both wheels
    leftWheel.update();
    rightWheel.update();

    // Get and print odometry data
    float leftOdometry = leftWheel.getOdometry();
    float rightOdometry = rightWheel.getOdometry();

    //Serial.print("Wheel 1 Odometry (radians): ");
    //Serial.print(leftOdometry);
    //Serial.print(",");
    //Serial.println(rightOdometry);

    //String Data = String(leftOdometry)+","+String(rightOdometry);
    globalOdom = String(leftOdometry)+","+String(rightOdometry);
    //transmit_data(Data);

    delay(20); // Small delay to reduce noise
  
}




void process_imu()
{
   if (!dmpReady) return;

  // Read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Send quaternion data over UDP
   // Send quaternion data over UDP
   char dataBuffer[100];
   //snprintf(dataBuffer, sizeof(dataBuffer), "quat,%f,%f,%f,%f", q.w, q.x, q.y, q.z);
   
   udp.beginPacket(remoteIP, UDP_PORT);

   //udp.printf("%f,%f,%f,%f",q.w,q.x,q.y,q.z);// Sends the data over wifi
   //udp.write((uint8_t*)dataBuffer, strlen(dataBuffer)); // Cast char* to uint8_t*
  //udp.endPacket();


    // Debug output to Serial
    //Serial.println(dataBuffer);
/*
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
*/

      float yaw = getYaw(q);
    yaw = adjustYaw(yaw);
    
   globalYaw = yaw;//q.z;
    //Serial.println(q.z);
  }
}

void heading_control()
{
   if (headingKeepingEnabled) {
            headingError = targetHeading - globalYaw;

            // Ensure the error is within -180 to 180 degrees
           // if (headingError > 180) headingError -= 360;
           // if (headingError < -180) headingError += 360;

            // PID calculations
            integral += headingError;
            float derivative = headingError - previousError;
            float correction = kp * headingError + ki * integral + kd * derivative;
            previousError = headingError;

            // Adjust motor speeds
            int leftCorrection = (Left_speed - (correction*10));
            int rightCorrection = (Right_speed + (correction*10));
            Serial.println(leftCorrection);

            //motor_speed(leftCorrection, rightCorrection); // Adjusted motor speeds
             LMotor.move(leftCorrection);
             RMotor.move(rightCorrection);
            String Debug = String(globalYaw) + "," + String(targetHeading) + "," + String(headingError) + "," + String(correction);
            //Serial.println(Debug);
           //transmit_data(Debug);
        }
  
}
void transmit_data(String Data)
{       //String Data = "Hello";
        udp.beginPacket(remoteIP, UDP_PORT);
        udp.write((const uint8_t*)Data.c_str(), Data.length());
        udp.endPacket();
}
void data_to_send()
{
  finalData = globalOdom + "," + String(globalYaw) + "," + String(globalDistance);
  transmit_data(finalData);
}

void measure_distance()
{
   uint16_t NewMeasurement;
  //Serial.print(".");
  if(sensor.readRangeNoBlocking(NewMeasurement)){
      //Serial.println(".");
        Serial.println(NewMeasurement);
        globalDistance = NewMeasurement;
}
}
float getYaw(Quaternion q) {
    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;
    
    float sinr_cosp = 2 * (w * z + x * y);
    float cosr_cosp = 1 - 2 * (y * y + z * z);
    return atan2(sinr_cosp, cosr_cosp);
}
float adjustYaw(float yaw) {
    if (yaw < 0) {
        yaw += TWO_PI;
    }
    return yaw;
}
