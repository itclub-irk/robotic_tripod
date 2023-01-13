#include <Ramp.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <CheapStepper.h>
#include <math.h>
#include <PID_v1.h>

#define loraRX D6
#define loraTX D5

rampDouble beaconAngleRamp;
int angleErrorRampTime = 200;

SoftwareSerial softSerial(loraTX, loraRX);
CheapStepper motor(D4, D3, D2, D1);

bool output_signed = true;
String inString = "";

int stepsInterval = 100;
unsigned long lastStepMillis = 0;

bool dir;

int currentTripodPosition = 0;
double currentAngleError = 0;
double pidOutput = 0;
double setpoint = 0;

double Kp=14, Ki=0.7, Kd=0;
PID pidRegulator(&currentAngleError, &pidOutput, &setpoint, Kp, Ki, Kd, REVERSE);

String inputString = "";

double prevBeaconAngle = 0;
double lastBeaconAngle = 0;
double expectedBeaconAngle = 0;

double angleSpeed = 0;
double constantAngleSpeed = 1;

float ZERO_ZONE = 0.6;

unsigned long lastBeaconCoordinatesHandled = 0;
long beaconCoordinatesHandleInterval = 200;

double angle;
boolean invertServo = false;

float STEPS_PER_DEGREE = 11.3778;

double toRadians(double angle) {
  return (angle * 71) / 4068;
}

double toDegrees(double angle) {
  return (angle * 4068) / 71;
}

int getCurrentTripodAngle() {
  return currentTripodPosition / STEPS_PER_DEGREE;
}

void ICACHE_RAM_ATTR onTimerISR(){

  if(stepsInterval != 100 && (millis() - lastStepMillis >= stepsInterval)){
    if(pidOutput > 0){
      dir = true;
    }else if(pidOutput < 0){
      dir = false;
    };

      if(dir){
        currentTripodPosition += 1;
      }else{
        currentTripodPosition -= 1;
      };  

    if(currentTripodPosition > 4096){
      currentTripodPosition = 0;
    }else if(currentTripodPosition < 0){
      currentTripodPosition = 4096;
    };

//    if(stepsRamp.getTarget() != 0){
//      stepsRamp.go(0, stepsRampTime);
//    };      
//    stepsInterval = stepsRamp.update();    

    motor.step(dir);  
    Serial.print("Steps interval: ");
    Serial.println(stepsInterval);
    Serial.print("Angle err: ");
    Serial.println(currentAngleError);

    lastStepMillis = millis();
  };
  timer1_write(5000);  
}

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

void setup() {   
  Serial.begin(115200);
  softSerial.begin(9600);
  
  WiFi.forceSleepBegin();
  delay(1); 

  pidRegulator.SetOutputLimits(-92, 92);
  pidRegulator.SetMode(AUTOMATIC);
  
  motor.step(dir); 
  
  timer1_attachInterrupt(onTimerISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  //2e-6
  timer1_write(5000);  
}

void loop() {
  handlePID();  
  recieveBeaconMessage();  
  handleTripodAngleError();
}

void handlePID(){
  pidRegulator.Compute();
  stepsInterval = round(100 - abs(pidOutput));
}

void recieveBeaconMessage() {

  while(softSerial.available()){   
    char inChar = softSerial.read();
    if(inChar == '\n'){
      handleBeaconMessage();
      inputString = "";    
    }else{
      inputString += inChar;
    }   
  };
}

void handleBeaconMessage() {
  String payload = getMessagePayload(inputString);
  String checksumm = getMessageChecksumm(inputString);

  if (checkPayloadIntegrity(payload, checksumm)) {
      prevBeaconAngle = lastBeaconAngle;
      lastBeaconAngle = payload.toInt() / 10.0;
      lastBeaconCoordinatesHandled = millis();  
      inputString = "";   
  };
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
    return msg.substring(payloadEndIndex + 1, msg.length() - 1);
  } else {
    return "";
  }
}

void handleTripodAngleError(){
    double currentTripodAngle = currentTripodPosition / STEPS_PER_DEGREE;
    
    double delta = lastBeaconAngle - currentTripodAngle;
    delta = modulo(delta + 180, 360) - 180;

    if (abs(delta) >= ZERO_ZONE) {
      currentAngleError = delta;
    }else{
      currentAngleError = 0;  
    };
}

double modulo(double a, double b) {
  return a - floor(a / b) * b;
}

