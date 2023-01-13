#include <ESP8266WiFi.h>
#include <math.h>
#include<AccelStepper.h>
#include <SPI.h>
#include <LoRa.h>

boolean DEBUG_MODE = false;

AccelStepper motor(1, 4, 5);

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

float STEPS_PER_DEGREE = 11.3778;

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

void rotateServo(int angle) {
  int steps = 0;
  if (invertServo) {
    steps = map(angle, 0, 360, 0, 4096);
  } else {
    steps = map(angle, 0, 360, 4096, 0);
  }
  motor.moveTo(steps);
}

int getCurrentTripodAngle() {
  return motor.currentPosition() / STEPS_PER_DEGREE;
}

void setAngleSpeed(float newSpeed) {
  if (newSpeed > 87.89) {
    newSpeed = 87.89;
  }
  float stepsPerSecond = map(newSpeed, 0, 87.89, 0, 1000);
  motor.setMaxSpeed(stepsPerSecond);
}

void setup() {
  motor.setMaxSpeed(1000.0); // Скорость вращения вала двигателя.
  motor.setAcceleration(600.0);

  rotateServo(45);
  while(motor.distanceToGo()!=0){
    motor.run();
  };
  delay(500);
  rotateServo(0);
  while(motor.distanceToGo()!=0){
    motor.run();
  };
  
  Serial.begin(9600);

//  if (DEBUG_MODE) {
//    Serial.println(F("Stepper test done"));
//  }
  
  WiFi.forceSleepBegin();
  delay(1); 

  LoRa.setPins(15, 16, LORA_DEFAULT_DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
}

void loop() {
  motor.run();
  recieveBeaconMessage();
  sendHeartbeat();
}

void recieveBeaconMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
//    Serial.print("RSSI: ");
//    Serial.println(LoRa.packetRssi());
    while (LoRa.available()) {
      int inChar = (char)LoRa.read();
      charRecieved += 1;
      if (charRecieved < 26) {
        if (inChar == '\n') {
          inputString = tmp;
          Serial.print("src: "+ inputString + " ");
          handleBeaconMessage();
          tmp = "";
          charRecieved = 0;
        } else {
          tmp += (char)inChar;
        }
      } else {
        tmp = "";
        charRecieved = 0;
      };
    };
  };
}

void sendHeartbeat(){
  if((millis() - lastHeartbeatSendedMillis) >= HEARTBEAT_INTERVAL){
    LoRa.beginPacket();
    LoRa.print("1");
    LoRa.endPacket(true);
    lastHeartbeatSendedMillis = millis();
  }    
}


void handleBeaconMessage() {
  String payload = getMessagePayload(inputString);
  String checksumm = getMessageChecksumm(inputString);
  
  if (checkPayloadIntegrity(payload, checksumm)) {
    prevBeaconAngle = lastBeaconAngle;
    lastBeaconAngle = payload.toInt() / 10;
    Serial.print("res: ");
    Serial.println(lastBeaconAngle);
    handleTripodAngleError();
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

double calculateAngleSpeed(){
  return (lastBeaconAngle - prevBeaconAngle) * 1000 / (millis() - lastBeaconCoordinatesHandled);
}

double calculateDeltaSpeed(double delta){
    return delta * 1000 / (millis() - lastBeaconCoordinatesHandled);
}

void handleTripodAngleError() {

    int currentTripodAngle = motor.currentPosition() / STEPS_PER_DEGREE;
    int delta = lastBeaconAngle - currentTripodAngle;
    delta = modulo(delta + 180, 360) - 180;
    
    if (abs(delta) > ZERO_ZONE) {
      
      angleSpeed = calculateAngleSpeed();
      
      
      setAngleSpeed(angleSpeed + calculateDeltaSpeed(delta));
      rotateServo(lastBeaconAngle);
    };
}

double modulo(double a, double b) {
  return a - floor(a / b) * b;
}

