#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define FIREBASE_HOST "robish-7ad30-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "AzIo9RNzEEVUTj2aRFWQgSBDj7sHJrYVVa705XHU"
#define WIFI_SSID "Mi 11 Lite" //provide ssid (wifi name)
#define WIFI_PASSWORD "95068953" //wifi password

WiFiServer server(80);


int rightfrontforward = 16;     /* GPIO2(D4) -> IN3   */
int rightfrontbackward = 5;   /* GPIO15(D8) -> IN1  */
int leftfrontforward = 4;    /* GPIO0(D3) -> IN4   */
int leftfrontbackward = 0;  /* GPIO13(D7) -> IN2  */
int rightbehindforward = 2;     /* GPIO2(D4) -> IN3   */
int rightbehindbackward = 14;   /* GPIO15(D8) -> IN1  */
int leftbehindforward = 12;    /* GPIO0(D3) -> IN4   */
int leftbehindbackward = 13;  /* GPIO13(D7) -> IN2  */


  

bool Status;



int trash = 15;

int obstacle = 9;

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

void setup() {
  // put your setup code here, to run once:
  pinMode(rightfrontforward, OUTPUT);
  pinMode(rightfrontbackward, OUTPUT);
  pinMode(leftfrontforward, OUTPUT);
  pinMode(leftfrontbackward, OUTPUT);
  pinMode(rightbehindforward, OUTPUT);
  pinMode(rightbehindbackward, OUTPUT);
  pinMode(leftbehindforward, OUTPUT);
  pinMode(leftbehindbackward, OUTPUT);

  Serial.begin(9600);

//    updateStatus(true);

  Status = false;

  pinMode(trash, INPUT);
  pinMode(obstacle, INPUT);

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
}


  




void MotorForward(void)
{
  digitalWrite(rightfrontbackward, LOW);
  digitalWrite(leftbehindbackward, LOW);
  digitalWrite(leftfrontbackward, LOW);
  digitalWrite(rightbehindbackward, LOW);

  digitalWrite(rightfrontforward, HIGH);
  

  digitalWrite(leftbehindforward, HIGH);

  digitalWrite(leftfrontforward, HIGH);
  

  digitalWrite(rightbehindforward, HIGH);
}

/***** BACKWARD *****/
void MotorBackward(void)
{
  digitalWrite(rightfrontforward, LOW);
  digitalWrite(rightfrontbackward, HIGH);
  digitalWrite(leftbehindbackward, HIGH);
  digitalWrite(leftbehindforward, LOW);
  digitalWrite(leftfrontforward, LOW);
  digitalWrite(leftfrontbackward, HIGH);
  digitalWrite(rightbehindbackward, HIGH);
  digitalWrite(rightbehindforward, LOW);
}

/***** LEFT *****/
void TurnLeft(void)
{
  digitalWrite(rightfrontforward, HIGH);
  digitalWrite(rightfrontbackward, LOW);
  digitalWrite(leftbehindbackward, HIGH);
  digitalWrite(leftbehindforward, LOW);
  digitalWrite(leftfrontforward, LOW);
  digitalWrite(leftfrontbackward, LOW);
  digitalWrite(rightbehindbackward, LOW);
  digitalWrite(rightbehindforward, LOW);
}

/***** RIGHT *****/
void TurnRight(void)
{
  digitalWrite(rightfrontforward, LOW);
  digitalWrite(rightfrontbackward, LOW);
  digitalWrite(leftbehindbackward, LOW);
  digitalWrite(leftbehindforward, LOW);
  digitalWrite(leftfrontforward, HIGH);
  digitalWrite(leftfrontbackward, LOW);
  digitalWrite(rightbehindbackward, HIGH);
  digitalWrite(rightbehindforward, LOW);
}

/***** STOP *****/
void MotorStop(void)
{
  digitalWrite(rightfrontforward, LOW);
  digitalWrite(rightfrontbackward, LOW);
  digitalWrite(leftbehindbackward, LOW);
  digitalWrite(leftbehindforward, LOW);
  digitalWrite(leftfrontforward, LOW);
  digitalWrite(leftfrontbackward, LOW);
  digitalWrite(rightbehindbackward, LOW);
  digitalWrite(rightbehindforward, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:   
  Serial.print("trash:");
  Serial.println(digitalRead(trash));
  Serial.print("obstacle:");
  Serial.println(digitalRead(obstacle));

  
  
MotorForward();

  if (digitalRead(trash)){
  TurnRight();
  delay(2000);
  MotorForward();
  delay(2000);
  TurnLeft();
  delay(2000);
  MotorForward();
  delay(2000);

  TurnLeft();
  delay(2000);
  MotorForward();
  delay(2000);
  TurnRight();
  delay(2000);
  MotorStop();
  delay(3000);
  
  
}
}
