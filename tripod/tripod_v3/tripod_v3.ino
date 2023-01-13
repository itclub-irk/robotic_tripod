#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <math.h>
#include<AccelStepper.h>
#define HALFSTEP 8
#include <WiFiUdp.h>

boolean DEBUG_MODE = false;

const char *ssid = "tripod";
const char *pass = "tripodtripod"; 

unsigned int localPort = 2000; // local port to listen for UDP packets

IPAddress serverIP(192,168,4,1);
IPAddress clientIP(192,168,4,2);

WiFiUDP udp;
char packetBuffer[20];

const int IN1 = 13;
const int IN2 = 12;
const int IN3 = 14;
const int IN4 = 16;

AccelStepper motor(HALFSTEP, IN1, IN3, IN2, IN4);

String inputString = "";
int charRecieved = 0;
String tmp = "";

int ledState = HIGH;
unsigned long ledOnMillis = 0;
const long ledInterval = 1000;

double prevBeaconAngle = 0;
double lastBeaconAngle = 0;

double angleSpeed = 0;
double constantAngleSpeed = 1;

int ZERO_ZONE = 1;

unsigned long lastBeaconCoordinatesHandled = 0;
long beaconCoordinatesHandleInterval = 200;

double angle;
boolean invertServo = true;

float STEPS_PER_DEGREE = 11.3778;

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

  rotateServo(90);
  while(motor.distanceToGo()!=0){
    motor.run();
  };
  delay(500);
  rotateServo(0);

  Serial.begin(115200);

  if (DEBUG_MODE) {
    Serial.println(F("Stepper test done"));
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  
  WiFi.softAP(ssid, pass);
  udp.begin(localPort);
}

void loop() {
  motor.run();
  recieveBeaconMessage();
  handleBuiltinLed();
}

void recieveBeaconMessage() {
  if(udp.parsePacket()){
    udp.read(packetBuffer, 20);
    inputString = String(packetBuffer);
    handleBeaconMessage();
  };
}

void handleBeaconMessage() {
  prevBeaconAngle = lastBeaconAngle;
  lastBeaconAngle = inputString.toInt() / 10;
  handleTripodAngleError();
  lastBeaconCoordinatesHandled = millis();
}

void handleBuiltinLed() {
  if (ledState == LOW) {
    if (millis() - ledOnMillis >= ledInterval) {
      ledState = HIGH;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
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

