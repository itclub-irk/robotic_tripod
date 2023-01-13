#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFiUdp.h>

#define BUTTON_PIN 4
#define VIB_PIN 5

const char *ssid = "tripod";
const char *pass = "tripodtripod"; 

unsigned int localPort = 2000;
 
IPAddress serverIP(192,168,4,1);
IPAddress clientIP(192,168,4,2);

WiFiUDP udp;
char packetBuffer[10];

static const int RXPin = 14, TXPin = 12;
static const uint32_t GPSBaud = 9600;
const uint32_t COMMAND_DELAY = 250;
TinyGPSPlus gps;

SoftwareSerial gpsPort(RXPin, TXPin); //Rx-D5, Tx-D6

unsigned long buttonPressedMillis = 0;
unsigned long powerOffPressedMillis = 0;
int buttonInterval = 1000;
int powerOffInterval = 3000;

boolean beaconEnabled = false;
boolean lastButtonState = false;
boolean actionFired = false;

int ledState = HIGH;
unsigned long ledOnMillis = 0;
const long interval = 1000;

String lastSendedMessage = "";
long lastMessageMillis = 0;
long sendMessageInterval = 200;

double defaultAngle;

double beaconLat;
double beaconLong;

double tripodLat;
double tripodLong;

double checkpointLat;
double checkpointLong;

double defaultAzimuth;
double beaconAzimuth;
double angle;

const int HEARTBEAT_INTERVAL = 3000;

boolean gpsReceived;
unsigned long lastHeartbeatCheckMillis;

double divider = 10000000;

const unsigned char ubxRate5Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,200,0x00,0x01,0x00,0x01,0x00 };

const unsigned char ubxDisableVTG[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01 };

const unsigned char ubxDisableRMC[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01 };

const unsigned char ubxDisableGSV[] PROGMEM =
  { 0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01 };

void sendUBX( const unsigned char *progmemBytes, size_t len ){
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

}

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

void initVariables(){
  lastSendedMessage = "";
  lastMessageMillis = 0;
  sendMessageInterval = 200;

  defaultAngle = 0;

  beaconLat = 0;
  beaconLong = 0;

  tripodLat = 0;
  tripodLong = 0;

  checkpointLat = 0;
  checkpointLong = 0;

  defaultAzimuth = 0;
  beaconAzimuth = 0;
  angle = 0;
  
  gpsReceived = false;
  lastHeartbeatCheckMillis = 0;  
}

extern "C" {
#include <cstdint>
#include "user_interface.h"
const  uint8_t mac[] = {0xFC, 0x95, 0xF2, 0x4E, 0x98, 0x0A};
}
bool changeMac(const uint8_t mac[6])
{
  return wifi_set_macaddr(STATION_IF, const_cast<uint8*>(mac));
}

void setup(){
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW); 
   
  Serial.begin(115200);
  gpsPort.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  lastHeartbeatCheckMillis = millis();

  initVariables();
  WiFi.mode(WIFI_STA);
  ESP.eraseConfig();
  changeMac(mac);
  WiFi.begin(ssid, pass);

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter < 5) {
    delay(500);
    counter += 1;
  }
    
  udp.begin(localPort);
}

void initExternalDevices(){
  sendUBX(ubxRate5Hz, sizeof(ubxRate5Hz));  
  delay(COMMAND_DELAY);
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  delay(COMMAND_DELAY);
  sendUBX(ubxDisableRMC, sizeof(ubxDisableRMC));  
  delay(COMMAND_DELAY);
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV)); 
}

void loop(){
  handleButton();
  handleGPS();
  handleHeartbeat(); 
}

void handleButton() {
  boolean currentButtonState = !digitalRead(BUTTON_PIN);
  
  if (currentButtonState && !lastButtonState) {
    buttonPressedMillis = millis();
    powerOffPressedMillis = millis();
    
    lastButtonState = currentButtonState;
  };
  if (!actionFired && currentButtonState && (millis() - buttonPressedMillis) >= buttonInterval) {
    ledState = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(VIB_PIN, HIGH);
    
    ledOnMillis = millis();
    handleCheckpoint();
    actionFired = true;
  };

//  power on / off
  if(currentButtonState && (millis() - powerOffPressedMillis) >= powerOffInterval){

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
      digitalWrite(VIB_PIN, LOW);      
    }
  }
}

void flashLED(int times, long period){
  for(int i = 0; i < times; i++){
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(VIB_PIN, HIGH);
    delay(period);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(VIB_PIN, LOW);
    delay(period);
  } 
}

void handleCheckpoint() {
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
}

static void handleGPS() {
  while (gpsPort.available() > 0){
    gps.encode(gpsPort.read());
    if (gps.location.isUpdated()){
      gpsReceived = true;
      beaconLat = gps.location.lat();
      beaconLong = gps.location.lng();
      handleBeaconCoordinates();
    };
  };
}

double modulo(double a, double b) {
  return a - floor(a / b) * b;
}

void handleBeaconCoordinates() {
  if (tripodLat != 0 && tripodLong != 0 && checkpointLat != 0 && checkpointLong != 0) {
    beaconAzimuth = getAzimuth(tripodLat, tripodLong, beaconLat, beaconLong);

    angle = beaconAzimuth - defaultAzimuth;
//    angle = modulo(angle + 180, 360) - 180;
    angle = modulo(angle, 360);
    lastSendedMessage = String(round(truncate(angle, 1) * 10));
    sendMessage(lastSendedMessage);
  }
}

void handleMessage(){
  if(lastSendedMessage != "" && (millis() - lastMessageMillis) >= sendMessageInterval){
    sendMessage(lastSendedMessage);
    lastMessageMillis = millis();
  }  
}

static void sendMessage(String msg) {
  char buff[20];
  
  udp.beginPacket(serverIP, localPort);
  msg.toCharArray(buff, 20);
  udp.write(buff, 20); 
  udp.endPacket();  
}

void handleHeartbeat(){
  if((millis() - lastHeartbeatCheckMillis ) >= HEARTBEAT_INTERVAL){
    
    if(WiFi.status() == WL_CONNECTED){
      if(gpsReceived){
        gpsReceived = false;
      }else{
        flashLED(1, 100);  
      };
    }else{
      flashLED(2, 100);
    };

    lastHeartbeatCheckMillis  = millis();
  }       
}

double truncate(double val, byte dec)
{
    double x = val * pow(10, dec);
    double y = round(x);
    double z = x - y;
    if ((int)z == 5)
    {
        y++;
    } else {}
    x = y / pow(10, dec);
    return x;
}
