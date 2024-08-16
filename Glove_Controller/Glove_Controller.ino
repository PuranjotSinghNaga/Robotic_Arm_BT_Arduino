#include <esp_now.h>
#include <Wifi.h>

// function prototype
float KALMAN(float U);


// Mac Address (add proper mac adress of the receiving end !!!)
uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x7A, 0xAE, 0x7C};
 


// Kalman filter state for each value
float P_ax = 0, U_Hat_ax = 0;
float P_ay = 0, U_Hat_ay = 0;
float P_az = 0, U_Hat_az = 0;
float P_gx = 0, U_Hat_gx = 0;
float P_gy = 0, U_Hat_gy = 0;
float P_gz = 0, U_Hat_gz = 0;

const float R = 0.33;   // noise covariance 
const float H = 1.00; // measurement map scalar
const float Q = 10;         // initial estimated covariance 

bool state_1 = true;


// final filtered values for ax, ay, az , gx, gy, gz
float filtered_aX
float filtered_aY
float filtered_aZ
float filtered_gX
float filtered_gY
float filtered_gZ


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

struct_message myData;

esp_now_peer_info_t peerInfo;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void setup() {
  // Setup data here
  Serial.begin(115200);
  WiFi.Mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  // 1. get values of state pin and mpu6050 sensor data 
  int16_t ax, ay, az, gx, gy, gz;
  // 2. get mpu values and add them to ax,ay,az,gx,gy,gz
  // 3. filter the values
  float filtered_ax = KALMAN(ax, P_ax, U_Hat_ax);
  float filtered_ay = KALMAN(ay, P_ay, U_Hat_ay);
  float filtered_az = KALMAN(az, P_az, U_Hat_az);
  float filtered_gx = KALMAN(gx, P_gx, U_Hat_gx);
  float filtered_gy = KALMAN(gy, P_gy, U_Hat_gy);
  float filtered_gz = KALMAN(gz, P_gz, U_Hat_gz);
  // 4. get state pin value


  // adding data to struct 
  myData.state = state_1;
  myData.aX = filtered_aX ;
  myData.aY = filtered_aY ;
  myData.aZ = filtered_aZ ;
  myData.gX = filtered_gX ;
  myData.gY = filtered_gY ;
  myData.gZ = filtered_gZ ;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
  delay(2000);
}

//function definations 

float KALMAN(float U) {
  static const float R = 0.33;   // noise covariance 
  static const float H = 1.00; // measurement map scalar
  static float Q = 10;         // initial estimated covariance 
  static float P = 0;          // initial error covariance (must be zero)
  static float U_Hat = 0;      // initial estimated state (assume we dont know)
  static float K = 0;          // initial Kalman Gain 


  K = P*H/(H*P*H+R);            // update Kalman Gain 
  U_Hat = U_Hat + K*(U-H*U_Hat);// update estimated state
  P=(1-K*H)*P+Q;                // update error covariance

  return U_Hat;                 // Return Final Value
};