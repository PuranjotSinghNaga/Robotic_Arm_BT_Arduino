#include "HCPA9685.h"

#define I2C_Add 0x40 // Default address for HCPA9685
#define dirPin 4     // Direction Pin for Stepper 
#define stepPin 5    // Step Pin for Stepper

/*...............DATA RECIEVING BLOCK ENDS...............*/

// message structure 
typedef struct struct_message {
  bool state;
  float aX; 
  float aY; 
  float aZ; 
  float gX; 
  float gY; 
  float gZ; 
} struct_message;

struct_message myData; // initialization

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("State Value: ");
  Serial.println(myData.state);
  Serial.print("Ax Value: ");
  Serial.println(myData.aX);
  Serial.print("Ay Value: ");
  Serial.println(myData.aY);
  Serial.print("Az Value: ");
  Serial.println(myData.aZ);
  Serial.print("Gx Value: ");
  Serial.println(myData.gX);
  Serial.print("Gy Value: ");
  Serial.println(myData.gY);
  Serial.print("Gz Value: ");
  Serial.println(myData.gZ);

}

/*...............DATA RECIEVING BLOCK ENDS...............*/



const int stepsPerRevolution = 120;       //Steps per revolution 
const int stepsPerRevolutionSmall = 60;   //Steps per revolution for small stepping 



HCPA9685 HCPA9685(I2CAdd); // initializing HCPA9685 Library

// Parking positions for Servos
const int servo_joint_L_parking_pos = 60; // base plate left
const int servo_joint_R_parking_pos = 60; // base plate right 
const int servo_joint_1_parking_pos = 70; // shoulder joint 
const int servo_joint_2_parking_pos = 47; // elbow joint 
const int servo_joint_3_parking_pos = 63; // wrist joint 1 
const int servo_joint_4_parking_pos = 63; // wrist joint 2

// Minimum and maximum angles for the Servos 
int servo_joint_L_min_pos = 10;
int servo_joint_L_max_pos = 180;

int servo_joint_R_min_pos = 10;
int servo_joint_R_max_pos = 180;

int servo_joint_1_min_pos = 10;
int servo_joint_1_max_pos = 400;

int servo_joint_2_min_pos = 10;
int servo_joint_2_max_pos = 380;

int servo_joint_3_min_pos = 10;
int servo_joint_3_max_pos = 380;

int servo_joint_4_min_pos = 10;
int servo_joint_4_max_pos = 120;


// Current Positions
int servo_L_pos = 0;
int servo_R_pos = 0;
int servo_joint_1_pos = 0;
int servo_joint_2_pos = 0;
int servo_joint_3_pos = 0;
int servo_joint_4_pos = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // put your main code here, to run repeatedly:

}
