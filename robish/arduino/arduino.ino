int ATBuzzerPin = 5; //D1 buzzer pin
int ATtrigP ;
int ATechoP;
int ObstacleTrigP;
int ObstacleEchoP;
int TrashTrigP  ;
int TrashEchoP  ;
long  duration;
float ATdistance;
float distanceTrash;
float distance;

int obstacle;
int trash;

int readfirebase;


int state;


void setup() {
  pinMode(ATtrigP, OUTPUT);
  pinMode(ATechoP, INPUT);

  pinMode(ObstacleTrigP, OUTPUT);
  pinMode(ObstacleEchoP, INPUT);
  pinMode(TrashTrigP, OUTPUT);
  pinMode(TrashEchoP, INPUT);

  pinMode(ATBuzzerPin, OUTPUT);
  pinMode(obstacle, OUTPUT);
  pinMode(obstacle, OUTPUT);

  pinMode(readfirebase, INPUT);


  //Set serial monitor
  Serial.begin(9600);
}

void loop() {
  if (digitalRead(readfirebase)) {
    antiTheft();
    if (checkObstacle()) {
      digitalWrite(obstacle, HIGH);
    }
    if (checkTrash()) {
      digitalWrite(trash, HIGH);
    }
  }

}




void antiTheft(void) {
  digitalWrite ( ATBuzzerPin, LOW);
  digitalWrite(ATtrigP, LOW);
  delayMicroseconds(2);       // 2 micro second delay
  digitalWrite(ATtrigP, HIGH);
  delayMicroseconds(10);      // trigPin high for 10 micro seconds
  digitalWrite(ATtrigP, LOW);
  duration = pulseIn(ATechoP, HIGH);   //Read echo pin, time in microseconds
  ATdistance = duration * 0.034 / 2;   //Calculating actual/real distance en cm

  Serial.println(ATdistance);

  if (ATdistance > 50)
  {
    digitalWrite ( ATBuzzerPin, HIGH);
    delay(1000);
    digitalWrite ( ATBuzzerPin, LOW);
  }

}

bool checkTrash(void) {
  digitalWrite(ObstacleTrigP, LOW);   // Makes trigPin low
  delayMicroseconds(2);       // 2 micro second delay
  digitalWrite(ObstacleTrigP, HIGH);  // tigPin high
  delayMicroseconds(10);      // trigPin high for 10 micro seconds
  digitalWrite(ObstacleTrigP, LOW);   // trigPin low
  duration = pulseIn(ObstacleEchoP, HIGH);   //Read echo pin, time in microseconds
  distance = duration * 0.034 / 2;   //Calculating actual/real distance en cm

  if (distance > 250 && checkObstacle() ) //and ObstacleTrigP,LOW & TrashtrigP,HIGH
  {
    return true;
  }

}

bool checkObstacle(void) {
  digitalWrite(TrashTrigP, LOW);   // Makes trigPin low
  delayMicroseconds(2);       // 2 micro second delay
  digitalWrite(TrashTrigP, HIGH);  // tigPin high
  delayMicroseconds(10);      // trigPin high for 10 micro seconds
  digitalWrite(TrashTrigP, LOW);   // trigPin low
  duration = pulseIn(TrashEchoP, HIGH);   //Read echo pin, time in microseconds
  distanceTrash = duration * 0.034 / 2;   //Calculating actual/real distance en cm

  if (distanceTrash < 100) //and ObstacleTrigP LOW
  {
    return true;
  }
}
