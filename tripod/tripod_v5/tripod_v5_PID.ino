#include <ESP8266WiFi.h>
#include <math.h>
#include <PID_v1.h>

#define stepPin 4
#define dirPin 5

bool output_signed = true;
String inString = "";

extern "C" {
#include <espnow.h>
}

#define CHANNEL 0

boolean DEBUG_MODE = false;

double Kp=15, Ki=0.5, Kd=0;
int stepsInterval = 300;
bool dir;
int currentTripodPosition = 0;
unsigned long lastStepMillis = 0;
double setpoint = 0;
double feedback = 0;
double output = 0;

PID myPID(&feedback, &output, &setpoint, Kp, Ki, Kd, REVERSE);

String inputString = "";
int charRecieved = 0;
String tmp = "";

double prevBeaconAngle = 0;
double lastBeaconAngle = 0;

double angleSpeed = 0;
double constantAngleSpeed = 1;

int ZERO_ZONE = 1;

unsigned long lastBeaconCoordinatesHandled = 0;
long beaconCoordinatesHandleInterval = 200;

const int HEARTBEAT_INTERVAL = 1000;
unsigned long lastHeartbeatSendedMillis = 0;

double angle;
boolean invertServo = true;

float STEPS_PER_DEGREE = 5.6889;

static const uint8_t PROGMEM dscrc_table[] = {
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95,  1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93,  3, 128, 222, 60, 98,
  190, 224,  2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89,  7,
  219, 133, 103, 57, 186, 228,  6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135,  4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91,  5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73,  8, 86, 180, 234, 105, 55, 213, 139,
  87,  9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

uint8_t crc8(const uint8_t *addr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
  }
  return crc;
}

double toRadians(double angle) {
  return (angle * 71) / 4068;
}

double toDegrees(double angle) {
  return (angle * 4068) / 71;
}

int getCurrentTripodAngle() {
  return currentTripodPosition / STEPS_PER_DEGREE;
}


void initESPNow() {
  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

void ICACHE_RAM_ATTR onTimerISR(){
  digitalWrite(stepPin, LOW);
  if(stepsInterval != 300 && (millis() - lastStepMillis >= stepsInterval)){
    if(output > 0){
      digitalWrite(dirPin, HIGH);
      dir = true;
    }else if(output < 0){
      digitalWrite(dirPin, LOW);
      dir = false;
    };
    
    digitalWrite(stepPin, HIGH);
    
    if(dir){
      currentTripodPosition += 1;
    }else{
      currentTripodPosition -= 1;
    }; 

    if(currentTripodPosition > 2048){
      currentTripodPosition = 0;
    }else if(currentTripodPosition < 0){
      currentTripodPosition = 2048;
    };
    lastStepMillis = millis();
  };
  timer1_write(5000);  
}

void setup() {
  myPID.SetOutputLimits(-297,297);
  myPID.SetMode(AUTOMATIC);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  lastStepMillis = millis();
  
  Serial.begin(9600);
  WiFi.mode(WIFI_AP_STA);
  initESPNow();
  esp_now_register_recv_cb(recieveBeaconMessage);

  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  //2e-6
  timer1_write(5000);  
}

void loop() {
  handlePID();
  handleTripodAngleError();
//  sendHeartbeat();
}

void handlePID(){
//  while (Serial.available() > 0) {
//    int inChar = Serial.read();
//    if(inChar != '\n'){
//      inString += (char)inChar;
//    }else{
//      boolean isPositive = true;
//      if(inString.substring(0, 1) == "-"){
//        isPositive = false;
//        inString.remove(0, 1);
//      };
//      lastBeaconAngle = inString.toFloat();
//      if(!isPositive){
//        lastBeaconAngle = -lastBeaconAngle;  
//      };
//      handleTripodAngleError();
//      inString = "";
//    }
//  };
  
  myPID.Compute();
  stepsInterval = round(300 - abs(output));
}

void recieveBeaconMessage(uint8_t *mac_addr, uint8_t *data, uint8_t len) {
  inputString = String((char*)data);
  handleBeaconMessage();
}

void sendHeartbeat(){
  if((millis() - lastHeartbeatSendedMillis) >= HEARTBEAT_INTERVAL){

    lastHeartbeatSendedMillis = millis();
  }    
}

void handleBeaconMessage() {
  String payload = getMessagePayload(inputString);
  String checksumm = getMessageChecksumm(inputString);
  
  if (checkPayloadIntegrity(payload, checksumm)) {
    prevBeaconAngle = lastBeaconAngle;
    lastBeaconAngle = payload.toInt() / 10.0;
    lastBeaconCoordinatesHandled = millis();
  };
}

String getMessagePayload(String msg) {
  if (msg.charAt(0) == '@') {
    int payloadEndIndex = msg.lastIndexOf("@");
    if (payloadEndIndex > 0) {
      return msg.substring(1, payloadEndIndex);
    }
  }
  return "";
}

String getMessageChecksumm(String msg) {
  int payloadEndIndex = msg.lastIndexOf('@');
  if (payloadEndIndex > 0) {
    return msg.substring(payloadEndIndex + 1);
  } else {
    return "";
  }
}

String hashData(String data) {
  return String(crc8((const uint8_t*)data.c_str(), data.length()));
}

boolean checkPayloadIntegrity(String payload, String checksumm) {
  if (payload != "" && checksumm != "") {
    String c = hashData(payload);
    if (c.equals(checksumm)) {
      return true;
    };
  }
  return false;
}

void handleTripodAngleError(){

    double currentTripodAngle = currentTripodPosition / STEPS_PER_DEGREE;
    double delta = lastBeaconAngle - currentTripodAngle;
    delta = modulo(delta + 180, 360) - 180;
        
    if (abs(delta) >= ZERO_ZONE) {
      feedback = delta;
    }else{
      feedback = 0;  
    };
}

double modulo(double a, double b) {
  return a - floor(a / b) * b;
}

