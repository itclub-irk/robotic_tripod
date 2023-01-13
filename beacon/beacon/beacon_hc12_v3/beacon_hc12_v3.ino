#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266HTTPClient.h>

#define BUTTON_PIN 4
#define POWER_ON_PIN 5
#define VIB_PIN 0
#define radioSerial Serial

const char* ssid     = "tripod";
const char* password = "tripodwifipass";

IPAddress ip(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

String serverHost = "http://192.168.4.1:80/handle_beacon_message";

static const int RXPin = 14, TXPin = 12;
static const uint32_t GPSBaud = 9600;
const uint32_t COMMAND_DELAY = 250;
TinyGPSPlus gps;
WiFiClient client;
SoftwareSerial gpsPort(RXPin, TXPin); //Rx-D5, Tx-D6

unsigned long buttonPressedMillis = 0;
int buttonInterval = 1000;

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

void setup(){
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(POWER_ON_PIN, LOW); 
  
  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW); 
   
  Serial.begin(115200); 
  Serial.println("Setup");  
  gpsPort.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  lastHeartbeatCheckMillis = millis();

  initVariables();
  
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
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
    lastButtonState = currentButtonState;
  };
  if (!actionFired && currentButtonState && (millis() - buttonPressedMillis) >= buttonInterval) {
    ledState = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(VIB_PIN, HIGH);
    
    ledOnMillis = millis();
     
    Serial.println("Handle checkpoint");
    handleCheckpoint();
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

    Serial.print("Tripod lat: ");
    Serial.println(tripodLat);
    Serial.print("Tripod long: ");
    Serial.println(tripodLong);    
  } else if (checkpointLat == 0 && checkpointLong == 0) {
    checkpointLat = beaconLat;
    checkpointLong = beaconLong;

    Serial.print("Tripod lat: ");
    Serial.println(checkpointLat);
    Serial.print("Tripod long: ");
    Serial.println(checkpointLong);     
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

static void sendMessage(String msg) {
  if (client.connect("http://192.168.4.1/", 80)) {
    String postData = "angle=" + msg;
    client.println("POST /handle_beacon_message HTTP/1.1");
    client.println("Host: 192.168.4.1");
    client.println("Cache-Control: no-cache");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.println(postData);
    client.stop();
    Serial.println(msg);
  }
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
