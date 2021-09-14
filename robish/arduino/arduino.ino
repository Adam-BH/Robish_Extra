int ATBuzzerPin = 8; //D1 buzzer pin

int ObstacleTrigP=7;
int ObstacleEchoP=6;
int TrashTrigP  =2;
int TrashEchoP  =3;

int induc;

long  duration;
float ATdistance;
float distanceTrash;
float distance;

int obstacle=9;
int trash=10;




int val; // define a numeric variable val 



void setup() {


  pinMode(ObstacleTrigP, OUTPUT);
  pinMode(ObstacleEchoP, INPUT);
  pinMode(TrashTrigP, OUTPUT);
  pinMode(TrashEchoP, INPUT);
  pinMode(ATBuzzerPin, OUTPUT);
  pinMode(obstacle, OUTPUT);
  pinMode(trash, OUTPUT);
  
  pinMode(induc, INPUT);


  //Set serial monitor
  Serial.begin(9600);

  
}

void loop() {
  if (val == HIGH ) {// when sensor detects shock,buzzer on 
    digitalWrite(ATBuzzerPin, HIGH);
  } 
  else {
    digitalWrite (ATBuzzerPin, LOW);
    delay(1000);
  }
    
    if (checkTrash()) {
      digitalWrite(trash, HIGH);
      
      Serial.println("trash detected");
    }
    else {
      Serial.println("trash not detected");
      
    }

    if (checkObstacle()) {
      digitalWrite(obstacle, HIGH);
      Serial.println("obstacle detected");
    }else {
      digitalWrite(obstacle, LOW);
      Serial.println("obstacle not detected");
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

  Serial.print("obstacle distance: ");
  Serial.println(distance);
  if (distance > 250 && checkObstacle() ) //and ObstacleTrigP,LOW & TrashtrigP,HIGH
  {
    return true;
  }else{
    return false;
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

  Serial.print("trash distance: ");
  Serial.println(distanceTrash);

  if (distanceTrash < 100) //and ObstacleTrigP LOW
  {
    return true;
  }else{
    return false;
  }
}
