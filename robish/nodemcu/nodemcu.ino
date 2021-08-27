#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Set these to run example.
#define FIREBASE_HOST "robish-7ad30-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "AzIo9RNzEEVUTj2aRFWQgSBDj7sHJrYVVa705XHU"
#define WIFI_SSID "Mi 11 Lite" //provide ssid (wifi name)
#define WIFI_PASSWORD "95068953" //wifi password

TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial ss(4, 5); // The serial connection to the GPS device PINS of GPS
WiFiServer server(80);

float longitudeS ; //Start point lat and long
float LatitudeS;
float longitudeE; //End point lat and long ---- GET IT FROM DATABASE
float LatitudeE ;

int leftMotorForward = 2;     /* GPIO2(D4) -> IN3   */
int rightMotorForward = 15;   /* GPIO15(D8) -> IN1  */
int leftMotorBackward = 0;    /* GPIO0(D3) -> IN4   */
int rightMotorBackward = 13;  /* GPIO13(D7) -> IN2  */

const int robotwidth = 0.18;
int upsn;
float upsw;
int i;
float Time;
int avoidt;
int avoidn;
float Speed;
int totallength;
int totalwidth;

int led;
int buzzer;

bool Status;

int(sendfirebase);

class Myclass {
  public:
    float lat;
    float lng;
};

class directions {
  public:
    bool backward;
    bool forward;
    bool left;
    bool right;
};


int obstacle;
int trash;



void setup() {
  pinMode(obstacle, INPUT);
  pinMode(trash, INPUT);
  pinMode(sendfirebase, OUTPUT);
  Serial.begin(9600);

  ss.begin(9600);


  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  Speed = ((2 * PI * 0.0325) / 60) * 240 / 60;
  avoidt = 10700;
  avoidn = 0;

  updateStatus(true);

  Status = false;
}

void autoPilot() {
  if (gps.location.isValid() || (readLocation().lat != 0 && isDigit(readLocation().lat))) {
    go();
  }
  else {
    digitalWrite(buzzer, HIGH);
  }
}

void go() {
  if (!Status){
    defineVariables();
  preparePath();
  }
  
  if (i <= upsn) {
    if ( millis() - Time - avoidn * avoidt < totalwidth / Speed * 1000) {
      checkObstacle(getLatitude(), getLongitude(), i);
      MotorForward();
    }
    else {
      turn(i);
    }
  }

}



void defineVariables() {
  LatitudeE = getLatitude() * 111.32 * 1000; //convert latitude to meters
  longitudeE = getLongitude() * 40075 * 1000 * cos( getLatitude() ) / 360; //convert longitude to meters
  LatitudeS = readDestination().lat * 111.32 * 1000;
  longitudeS = readDestination().lng * 40075 * 1000 * cos( readDestination().lat ) / 360;
}

float getLatitude() {
  if (gps.location.isValid() )
  {
    return (gps.location.lat());


  }
  else {
    return readLocation().lat;
  }
}

float getLongitude() {
  if (gps.location.isValid() )
  {
    return (gps.location.lng());


  }
  else {
    return readLocation().lng;
  }
}


void preparePath() {
  Time = millis();
  totallength = floor (abs(LatitudeE - LatitudeS));
  totalwidth = floor (abs(longitudeE - longitudeS));
  int upsn = floor(floor(totallength / robotwidth) * 0.7);

  if (upsn % 2 != 0 ) {
    upsn = upsn - 1;
  }
  upsw = floor(totallength / upsn);
  i = 0;
  Status = true;
}

void turn(int i) {
  if (startsRight()) {
    if (i % 2 == 0) {
      TurnRight();
      delay(400);
      MotorForward();
      delay(floor(upsw / Speed) * 1000);
      TurnRight();
      delay(400);
    }
    else {
      TurnLeft();
      delay(400); MotorForward();
      MotorForward();
      delay(floor(upsw / Speed) * 1000);
      TurnLeft();
      delay(400);
    }
  }
  else {
    if (i % 2 != 0) {
      TurnRight();
      delay(400);
      MotorForward();
      delay(floor(upsw / Speed) * 1000);
      TurnRight();
      delay(400);
    }
    else {
      TurnLeft();
      delay(400); MotorForward();
      MotorForward();
      delay(floor(upsw / Speed) * 1000);
      TurnLeft();
      delay(400);
    }
  }
  Time = millis();
  avoidn = 0;
  i++;
}


void checkObstacle (float lat, float lng, int i) {
  if (digitalRead(obstacle) == HIGH) {
    checkTrash(lat, lng);
    avoidObstacle(i);
  }
}

void checkTrash(float lat, float lng) {
  if (digitalRead(trash) == HIGH) {
    pushTrash(lat, lng);
  }
}

void avoidObstacle(int i) {
  if (startsRight()) {
    if (i % 2 == 0) {
      AvoidRight();
    }
    else {
      AvoidLeft();

    }
  }
  else {
    if (i % 2 != 0) {
      AvoidRight();
    }
    else {
      AvoidLeft();
    }
  }
}
bool startsRight() {
  if ((LatitudeE > LatitudeS && longitudeE < longitudeS) || (LatitudeE < LatitudeS && longitudeE > longitudeS)) {
    return true;
  }
  else {
    return false;
  }
}

void AvoidLeft (void)
{
  MotorStop();
  delay(100);
  TurnLeft();
  delay(400);
  MotorForward();
  delay(3000);
  TurnRight();
  delay(400);
  MotorForward();
  delay(3000);
  TurnRight();
  delay(400);
  MotorForward();
  delay(3000);
  TurnLeft();
  delay(400);
  avoidn++;
  //  MotorForward();
  //  delay (floor(totallength - (sqrt((longitudeO - longitudeS)* * 2 - (latitudeO - latitudeS)* * 2));
  //               // distance totale - (distance obs+distance done) =>
}

void AvoidRight(void) {

  MotorStop();
  delay(100);
  TurnRight();
  delay(400);
  MotorForward();
  delay(3000);
  TurnLeft();
  delay(400);
  MotorForward();
  delay(3000);
  TurnLeft();
  delay(400);
  MotorForward();
  delay(3000);
  TurnRight();
  delay(400);
  avoidn++;
  //  MotorForward();
  //  delay (floor(totallength - (sqrt((longitudeO - longitudeS)* * 2 - (latitudeO - latitudeS)* * 2));
  //               // distance totale - (distance obs+distance done) =>
}

void updateBattery(float battery) {
  Firebase.setFloat("/robots/KSAsjaAKS/battery/percentage", battery);
}

Myclass readDestination() {
  Myclass myObj;
  myObj.lat = Firebase.getFloat("/robots/KSAsjaAKS/destination/lat");
  myObj.lng = Firebase.getFloat("/robots/KSAsjaAKS/destination/lng");
  return myObj;
}

void updateDestination(float lat, float lng) {
  Firebase.setFloat("/robots/KSAsjaAKS/destination/lat", lat);
  Firebase.setFloat("/robots/KSAsjaAKS/destination/lng", lng);
}

bool readManual() {
  return Firebase.getBool("/robots/KSAsjaAKS/manual") == true || Firebase.getString("/robots/KSAsjaAKS/manual") == "true";
}
void updateManual(bool manual) {
  Firebase.setBool("/robots/KSAsjaAKS/,manual", manual);
}
directions readDirections() {
  directions myObj;
  myObj.backward = Firebase.getString("/robots/KSAsjaAKS/directions/backward") == "true" || Firebase.getBool("/robots/KSAsjaAKS/directions/backward") == true;
  myObj.forward = Firebase.getString("/robots/KSAsjaAKS/directions/forward") == "true" || Firebase.getBool("/robots/KSAsjaAKS/directions/forward") == true;
  myObj.left = Firebase.getString("/robots/KSAsjaAKS/directions/left") == "true" || Firebase.getBool("/robots/KSAsjaAKS/directions/left") == true;
  myObj.right = Firebase.getString("/robots/KSAsjaAKS/directions/right") == "true" || Firebase.getBool("/robots/KSAsjaAKS/directions/right") == true;
  return myObj;
}

void updateLocation(float lat, float lng) {
  Firebase.setFloat("/robots/KSAsjaAKS/location/lat", lat);
  Firebase.setFloat("/robots/KSAsjaAKS/location/lng", lng);
}

Myclass readLocation() {
  Myclass myObj;
  myObj.lat = Firebase.getFloat("/robots/KSAsjaAKS/location/lat");
  myObj.lng = Firebase.getFloat("/robots/KSAsjaAKS/location/lng");
  return myObj;
}

void updateScanning(bool scanning) {
  Firebase.setBool("/robots/KSAsjaAKS/scanning", scanning);
}

bool readScanning() {
  return Firebase.getBool("/robots/KSAsjaAKS/scanning") == true || Firebase.getString("/robots/KSAsjaAKS/scanning") == "true";
}

void updateStatus(bool state) {
  Firebase.setBool("/robots/KSAsjaAKS/status", state);
}

bool readStatus() {
  return Firebase.getBool("/robots/KSAsjaAKS/status") == true || Firebase.getString("/robots/KSAsjaAKS/status") == "true";
}

void pushTrash(float lat, float lng) {
  DynamicJsonBuffer jBuffer;
  JsonObject& root = jBuffer.createObject();
  root["lat"] = lat;
  root["lng"] = lng;
  String name = Firebase.push("/robots/KSAsjaAKS/harmful", root);


  root["accuracy"] = true;
  name = Firebase.push("/robots/KSAsjaAKS/harmless", root);
  if (Firebase.failed()) {
    Serial.print("pushing /harmful failed:");
    Serial.println(Firebase.error());
    return;
  }
  Serial.print("pushed: /harmful/");
  Serial.println(name);
  delay(1000);
}



void loop() {


  if (readStatus()) {
  digitalWrite(sendfirebase,HIGH);

    if (readScanning()) {
      autoPilot();


    }
    else {
      Status = false;
      if (readManual()) {
        if (readDirections().forward || readDirections().backward || readDirections().left || readDirections().right) {
          if (readDirections().forward) {
            MotorForward();
          }
          if (readDirections().backward) {
            MotorBackward();
          }
          if (readDirections().left) {
            TurnLeft();
          }
          if (readDirections().right) {
            TurnRight();
          }
        }
        else {
          MotorStop();
        }
      }
    }
    digitalWrite(led, HIGH);
  }
  else{
    Status = false;
    digitalWrite(sendfirebase,LOW);
  }

}

void MotorForward(void)
{
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

/***** BACKWARD *****/
void MotorBackward(void)
{
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorForward, LOW);
}

/***** LEFT *****/
void TurnLeft(void)
{
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
}

/***** RIGHT *****/
void TurnRight(void)
{
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
}

/***** STOP *****/
void MotorStop(void)
{
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}
