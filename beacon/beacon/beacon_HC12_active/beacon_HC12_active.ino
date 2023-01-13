#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <NMEAGPS.h>
#include <Streamers.h>

#define gpsPort Serial
#define DEBUG_PORT Serial
#define USING_GPS_PORT "Serial"
#define GPS_PORT_NAME "Serial"
#define BUTTON_PIN 4
#define POWER_ON_PIN 5
#define VIB_PIN 0
#define UPDATE_RATIO_HZ 5

static const uint8_t PROGMEM dscrc_table[] = {
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
   35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
   70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
   17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
   50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
   87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

const unsigned char ubxRate5Hz[] PROGMEM =
  { 0x06,0x08,0x06,0x00,200,0x00,0x01,0x00,0x01,0x00 };

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

uint8_t crc8(const uint8_t *addr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
  }
  return crc;
}

static NMEAGPS  gps;
static gps_fix  fix;

unsigned long buttonPressedMillis = 0;
unsigned long powerOffPressedMillis = 0;
int buttonInterval = 1000;
int powerOffInterval = 3000;

boolean lastButtonState = false;
boolean actionFired = false;
boolean runOnce = false;

int ledState = HIGH;
unsigned long ledOnMillis = 0;
const long interval = 1000;

String lastSendedMessage = "";
long lastMessageMillis = 0;
long sendMessageInterval = 200;

double defaultAngle = 0;

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
boolean heartbeatReceived = false;
boolean gpsReceived = false;
unsigned long lastHeartbeatCheckMillis = 0;

SoftwareSerial radioSerial(14, 12); //Rx-D5, Tx-D6

const char baud38400 [] PROGMEM = "PUBX,41,1,3,3,38400,0";

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

void setup() {
//  ESP.wdtDisable();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(POWER_ON_PIN, HIGH); 
  
  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW); 
  radioSerial.begin(9600);
  
  gpsPort.begin(9600);
  sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) );
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  lastHeartbeatCheckMillis = millis();

  WiFi.forceSleepBegin();
  delay(1);
}

void loop() {
  runOnceAction();
  handleGPS();
  handleButton();
//  handleMessage();
  handleHeartbeat();
}

void runOnceAction(){
  if(!runOnce){
    delay(500);
//    if(!digitalRead(BUTTON_PIN)){
//      digitalWrite(POWER_ON_PIN, LOW);     
//    }else{
//      flashLED(3, 200);
//    };
    flashLED(3, 200);
    runOnce = true;
  }  
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

//  power off
  if(currentButtonState && (millis() - powerOffPressedMillis) >= powerOffInterval){
    flashLED(3, 200);
//    digitalWrite(POWER_ON_PIN, LOW);
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
  while (gps.available( gpsPort )) {
    fix = gps.read();
    beaconLat = fix.latitudeL() / divider;
    beaconLong = fix.longitudeL() / divider;
    handleBeaconCoordinates();
  }
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
  String m;
  m = '@' + msg + '@' + hashData(msg) + '\n';
  radioSerial.println(m);
}

String hashData(String data) {
  return String(crc8((const uint8_t*)data.c_str(), data.length()));
}

void handleHeartbeat(){
  if(radioSerial.available() > 0){
    heartbeatReceived = true;
    while (radioSerial.available() > 0) {
      radioSerial.read();
    };    
  };

  if((millis() - lastHeartbeatCheckMillis ) >= HEARTBEAT_INTERVAL){
    if(heartbeatReceived){
      heartbeatReceived = false;
//      digitalWrite(LED_BUILTIN, HIGH); 
    }else{
//      digitalWrite(LED_BUILTIN, LOW);
      flashLED(2, 100);
    };
//    flashLED(2, 100);
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

