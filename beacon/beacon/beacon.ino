#include <NMEAGPS.h>
#include <ESP8266WiFi.h>

#define gpsPort Serial
#define DEBUG_PORT Serial
#define USING_GPS_PORT "Serial"
#define GPS_PORT_NAME "Serial"
#define BUTTON_PIN 5
#include <Streamers.h>
static NMEAGPS  gps;
static gps_fix  fix;
const char *ssid = "tripod";
const char *password = "tripodtripod";

boolean lastButtonState = false;

static void sendButtonData(){
  WiFiClient client;
  const char * host = "192.168.4.1";
  const int httpPort = 80;
  
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  String url = "/button/";

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }      
}

static void sendGPSData()
{
  double divider = 10000000;

//  trace_all( DEBUG_PORT, gps, fix );
  DEBUG_PORT.println(' ');
  DEBUG_PORT.print("Lat as long: ");
  DEBUG_PORT.println(fix.latitudeL());
  DEBUG_PORT.print("Lat as float: ");
  printDouble(fix.latitudeL() / divider, 7);
  Serial.println(" ");

  DEBUG_PORT.print("Long as long: ");
  DEBUG_PORT.println(fix.longitudeL());
  DEBUG_PORT.print("Long as float: ");
  printDouble(fix.longitudeL() / divider, 7);  
  Serial.println(" ");

  WiFiClient client;
  const char * host = "192.168.4.1";
  const int httpPort = 80;

  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  // We now create a URI for the request. Something like /data/?sensor_reading=123
  String url = "/beacon/";
  url += "?lat=";
  url += fix.latitudeL();
  url += "&long=";
  url += fix.longitudeL();

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }  
} 

void printDouble( double val, byte precision){
 Serial.print (int(val));  //prints the int part
 if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
      mult *=10;
      
   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
     padding--;
   while(  padding--)
     Serial.print("0");
   Serial.print(frac,DEC) ;
 }
}

static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    sendGPSData();
  }

} 

void handleButton(){
  boolean currentButtonState = !digitalRead(BUTTON_PIN);
  if(currentButtonState && !lastButtonState){
    lastButtonState = true;
    sendButtonData();
  }else if(!currentButtonState && lastButtonState){
    lastButtonState = false; 
  }
}

void setup()
{
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT);
  DEBUG_PORT.flush();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  gpsPort.begin( 9600 );

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }  
}

void loop()
{
  GPSloop();
  handleButton();  
}
