
#include <FastCRC.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <NMEAGPS.h>
#include <Streamers.h>

#define gpsPort Serial
#define DEBUG_PORT Serial
#define USING_GPS_PORT "Serial"
#define GPS_PORT_NAME "Serial"
#define BUTTON_PIN 4

FastCRC8 CRC8;

static NMEAGPS  gps;
static gps_fix  fix;

unsigned long buttonPressedMillis = 0;
int buttonInterval = 2000;

boolean lastButtonState = false;
boolean actionFired = false;

int ledState = HIGH;
unsigned long ledOnMillis = 0;
const long interval = 1000;

String lastSendedMessage = "";

SoftwareSerial radioSerial(14, 12); //Rx-D5, Tx-D6

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  radioSerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);

  WiFi.forceSleepBegin();
  delay(1);
}

void loop() {
  handleButton();
  handleGPS();
}

void handleButton() {
  boolean currentButtonState = !digitalRead(BUTTON_PIN);
  if (currentButtonState && !lastButtonState) {
    buttonPressedMillis = millis();
    lastButtonState = currentButtonState;
  };
  if (!actionFired && currentButtonState && (millis() - buttonPressedMillis) >= buttonInterval) {
    ledState = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    ledOnMillis = millis();
    sendCheckpointMessage();
    actionFired = true;
  };
  if (!currentButtonState && lastButtonState) {
    lastButtonState = currentButtonState;
    actionFired = false;
  };

  if (ledState == LOW) {
    unsigned long currentMillis = millis();
    if (currentMillis - ledOnMillis >= interval) {
      ledState = HIGH;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
}

void sendCheckpointMessage() {
  sendMessage("c");
}

static void handleGPS() {
  while (gps.available( gpsPort )) {
    fix = gps.read();
    sendGPSData();
  }
}

static void sendGPSData(){
  sendMessage(String(fix.latitudeL()) + ";" + String(fix.longitudeL()));
}

static void sendMessage(String msg){
  lastSendedMessage = '@' + msg + '@' + hashData(msg) + '\n';
  radioSerial.print(lastSendedMessage);
}

static String hashData(String data){
  uint8_t buf[15];
  data.getBytes(buf, data.length());
  return String(CRC8.smbus(buf, data.length()), HEX);
}


