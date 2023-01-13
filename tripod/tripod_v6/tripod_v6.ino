#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#include <math.h>
#include <PID_v1.h>

#define stepPin 4
#define dirPin 5
#define loraRX D6
#define loraTX D5

SoftwareSerial softSerial(loraTX, loraRX);

bool output_signed = true;
String inString = "";

double Kp=14, Ki=0.7, Kd=0;
//double Kp=10, Ki=1.5, Kd=1;
int stepsInterval = 300;
bool dir;
int currentTripodPosition = 0;
unsigned long lastStepMillis = 0;
double setpoint = 0;
double feedback = 0;
double output = 0;

PID myPID(&feedback, &output, &setpoint, Kp, Ki, Kd, REVERSE);

String inputString = "";

double prevBeaconAngle = 0;
double lastBeaconAngle = 0;

double angleSpeed = 0;
double constantAngleSpeed = 1;

float ZERO_ZONE = 1;

unsigned long lastBeaconCoordinatesHandled = 0;
long beaconCoordinatesHandleInterval = 200;

double angle;
boolean invertServo = false;

float STEPS_PER_DEGREE = 8.8889;

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
  softSerial.begin(9600);
  
  WiFi.forceSleepBegin();
  delay(1); 

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
  myPID.Compute();
  stepsInterval = round(300 - abs(output));
}

void recieveBeaconMessage() {
  while(softSerial.available()){   
    char inChar = softSerial.read();
    if(inChar == '\n'){
      prevBeaconAngle = lastBeaconAngle;
      lastBeaconAngle = inputString.toInt() / 10.0;
      lastBeaconCoordinatesHandled = millis();  
      inputString = "";    
    }else{
      inputString += inChar;
    }   
  };
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

