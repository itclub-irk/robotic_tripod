#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <math.h>
#include <Servo.h>
#include <FastCRC.h>
#include<AccelStepper.h>
#define HALFSTEP 8

FastCRC8 CRC8;

String inputString = "";
int charRecieved = 0;
String tmp = "";

int ledState = HIGH;
unsigned long ledOnMillis = 0;
const long ledInterval = 1000;

double beaconLat;
double beaconLong;

double tripodLat;
double tripodLong;

double checkpointLat;
double checkpointLong;

double defaultAzimuth;
double defaultAngle = 90;

double beaconAzimuth;
double angle;
boolean invertServo = true;
Servo servo;

//SoftwareSerial radioSerial(14, 12); //Rx-D5, Tx-D6

double divider = 10000000;

double toRadians(double angle) {
  return (angle * 71) / 4068;
}

double toDegrees(double angle) {
  return (angle * 4068) / 71;
}

double getAzimuth(double p1lat, double p1long, double p2lat, double p2long) {
  p1lat = toRadians(p1lat);
  p1long = toRadians(p1long);
  p2lat = toRadians(p2lat);
  p2long = toRadians(p2long);

  double delta_long = p2long - p1long;
  double y = sin(delta_long) * cos(p2lat);
  double x = cos(p1lat) * sin(p2lat) - sin(p1lat) * cos(p2lat) * cos(delta_long);
  double a = atan2(y, x);
  a = toDegrees(a);

  if (a < 0) {
    return a + 360;
  } else {
    return a;
  }
}

void rotateServo(int angle) {
  if (invertServo) {
    angle = 180 - angle;
  };
  if (angle > 180) {
    angle = 180;
  };
  if (angle < 0) {
    angle = 0;
  };
  servo.write(angle);
}

void setup() {
  //  servo.attach(5, 800, 2740);
  servo.attach(5, 685, 2300);
  rotateServo(0);
  delay(1500);
  rotateServo(180);
  delay(1500);
  rotateServo(defaultAngle);

  Serial.begin(9600);
//  radioSerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);

  WiFi.forceSleepBegin();
  delay(1);
}

void loop() {
  recieveBeaconMessage();
  handleBeaconCoordinates();
  handleBuiltinLed();
}

void recieveBeaconMessage() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    charRecieved += 1;

    if (charRecieved < 26) {
      if (inChar == '\n') {
        inputString = tmp;
        handleBeaconMessage();
        tmp = "";
        charRecieved = 0;
      } else {
        tmp += (char)inChar;
      }
    }else{
        tmp = "";
        charRecieved = 0;
    }
  }
}

void handleBeaconMessage() {

//  Serial.print("received: ");
//  Serial.println(inputString);

  String payload = getMessagePayload(inputString);
  String checksumm = getMessageChecksumm(inputString);

  if (checkPayloadIntegrity(payload, checksumm)) {
//    Serial.print("payload: ");
//    Serial.println(payload);

    if (payload == "c") {
      if (tripodLat == 0 && tripodLong == 0) {
        tripodLat = beaconLat;
        tripodLong = beaconLong;
      } else if (checkpointLat == 0 && checkpointLong == 0) {
        checkpointLat = beaconLat;
        checkpointLong = beaconLong;

        if (tripodLat != 0 && tripodLong != 0 && defaultAzimuth == 0) {
          defaultAzimuth = getAzimuth(tripodLat, tripodLong, checkpointLat, checkpointLong);
        }
      }

      ledOnMillis = millis();
      ledState = LOW;
      digitalWrite(LED_BUILTIN, ledState);
    } else {
      char atol_buffer[15];
      getValue(payload, ';', 0).toCharArray(atol_buffer, sizeof(atol_buffer));
      beaconLat = atol(atol_buffer) / divider;

      getValue(payload, ';', 1).toCharArray(atol_buffer, sizeof(atol_buffer));
      beaconLong = atol(atol_buffer) / divider;
    }
  } else {
    Serial.println("Broken message, reject");
  }
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
  uint8_t buf[15];
  data.getBytes(buf, data.length());
  return String(CRC8.smbus(buf, data.length()), HEX);
}

boolean checkPayloadIntegrity(String payload, String checksumm) {
  if (payload != "" && checksumm != "") {
    String c = hashData(payload);
    if (c == checksumm) {
      return true;
    };
  }
  return false;
}

void handleBuiltinLed() {
  if (ledState == LOW) {
    if (millis() - ledOnMillis >= ledInterval) {
      ledState = HIGH;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
}

double modulo(double a, double b) {
  return a - floor(a / b) * b;
}

void handleBeaconCoordinates() {
  if (tripodLat != 0 && tripodLong != 0 && checkpointLat != 0 && checkpointLong != 0) {
    beaconAzimuth = getAzimuth(tripodLat, tripodLong, beaconLat, beaconLong);
//    Serial.print("default az: ");
//    Serial.println(defaultAzimuth);
//    Serial.print("beacon az: ");
//    Serial.println(beaconAzimuth);

    angle = beaconAzimuth - defaultAzimuth;
    angle = modulo(angle + 180, 360) - 180;

//    Serial.print("angle: ");
//    Serial.println(angle);

    rotateServo((int)(defaultAngle + angle + .51));
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


