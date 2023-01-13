#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <math.h>
#include <Servo.h>

const char *ssid = "tripod";
const char *password = "tripodtripod";

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
Servo servo;

double divider = 10000000;

ESP8266WebServer server(80);

double toRadians(double angle){
  return (angle * 71) / 4068;
}

double toDegrees(double angle){
  return (angle * 4068) / 71;
}

double getAzimuth(double p1lat, double p1long, double p2lat, double p2long){
  p1lat = toRadians(p1lat);
  p1long = toRadians(p1long);
  p2lat = toRadians(p2lat);
  p2long = toRadians(p2long);
  
  double delta_long = p2long - p1long;
  double y = sin(delta_long) * cos(p2lat);
  double x = cos(p1lat) * sin(p2lat) - sin(p1lat) * cos(p2lat) * cos(delta_long);
  double a = atan2(y, x);
  a = toDegrees(a);
  
  if(a < 0){
    return a + 360;  
  }else{
    return a;
  }
}

void handleSentVar() {
  if (server.hasArg("lat") && server.hasArg("long")) { // this is the variable sent from the client
    char atol_buffer[15];

    server.arg("lat").toCharArray(atol_buffer, sizeof(atol_buffer));
    beaconLat = atol(atol_buffer) / divider;

    server.arg("long").toCharArray(atol_buffer, sizeof(atol_buffer));
    beaconLong = atol(atol_buffer) / divider;

    if(tripodLat != 0 && tripodLong != 0 && checkpointLat != 0 && checkpointLong != 0){
      beaconAzimuth = getAzimuth(tripodLat, tripodLong, beaconLat, beaconLong);
      angle = beaconAzimuth - defaultAzimuth;
      rotateServo((int)(defaultAngle + angle + .51));
    }

    server.send(200, "text/html", "Data received");
  }
}

void rotateServo(int angle){
  servo.write(180 - angle);
}

void displayInfo(){
  String result = "<h1>Last beacon position</h1><p>Lat: ";
  result += beaconLat * 10000000;
  result += "</p><p>Long: ";
  result += beaconLong * 10000000;
  result += "</p><h1>Tripod position</h1><p>Lat: ";
  result += tripodLat * 10000000;
  result += "</p><p>Long: ";
  result += tripodLong * 10000000;
  result += "</p><p><h1>Checkpoint position</h1><p>Lat: ";
  result += checkpointLat * 10000000;
  result += "</p><p>Long: ";
  result += checkpointLong * 10000000;
  result += "</p><h1>Azimuths</h1><p>Default azimuth: ";
  result += defaultAzimuth;
  result += "</p><p>Beacon azimuth: ";
  result += beaconAzimuth;
  result += "</p><p>Angle: "; 
  result += angle;
  server.send(200, "text/html", result);
}

void handleButtonData(){
  if(tripodLat == 0 && tripodLong == 0){
    setTripodCoordinates();
  }else if(checkpointLat == 0 && checkpointLong == 0){
    setCheckpointCoordinates();
  }
}

void setTripodCoordinates(){
  tripodLat = beaconLat;
  tripodLong = beaconLong;
  server.send(200, "text/html", "<p>Tripod coordinates setted</p>");
}

void setCheckpointCoordinates(){
  checkpointLat = beaconLat;
  checkpointLong = beaconLong;  

  if(tripodLat != 0 && tripodLong != 0 && defaultAzimuth == 0){   
    defaultAzimuth = getAzimuth(tripodLat, tripodLong, checkpointLat, checkpointLong); 
  }  
  server.send(200, "text/html", "<p>Checkpoint coordinates setted</p>");
}

void setup() {
  servo.attach(2);
  rotateServo(0);
  delay(1500);
  rotateServo(180);
  delay(1500);
  rotateServo(defaultAngle);

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();

  server.on("/beacon/", HTTP_GET, handleSentVar); 
  server.on("/info/", HTTP_GET, displayInfo); 
  server.on("/button/", HTTP_GET, handleButtonData);   
  server.on("/set_tripod_coordinates/", HTTP_GET, setTripodCoordinates);
  server.on("/set_checkpoint_coordinates/", HTTP_GET, setCheckpointCoordinates);
  server.begin();
}

void loop() {
  server.handleClient();
}
