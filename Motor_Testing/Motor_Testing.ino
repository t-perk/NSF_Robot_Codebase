#define ENA 5 // Right wheels speed controller
#define ENB 6 // Left wheels speed controller
#define IN1 3 // right
#define IN2 4 // right
#define IN3 2 // left
#define IN4 7 // left

#define carSpeed 130//Carspeed can be between 0-255

void forward(){//forward function
  analogWrite(ENA, carSpeed);//Set the speed of ENA
  analogWrite(ENB, carSpeed);//Set the speed of ENB
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Forward");

}

void back() {//back function
  analogWrite(ENA, carSpeed);//Set the speed of ENA
  analogWrite(ENB, carSpeed);//Set the speed of ENB
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Back");
}

void left() {//left function
  analogWrite(ENA, 200);//Set the speed of ENA
  analogWrite(ENB, 200);//Set the speed of ENB
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Left");
}

void right() {//right function
  analogWrite(ENA, 200);//Set the speed of ENA
  analogWrite(ENB, 200);//Set the speed of ENB
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {//stop function
  digitalWrite(ENA, LOW);//Set the speed of ENA to low 
  digitalWrite(ENB, LOW);//Set the speed of ENB to low
  Serial.println("Stop!");
}

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
}

void loop(){

  analogWrite(ENA, carSpeed);//Set the speed of ENA

  // Right forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  delay(3000);

  // Right backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  delay(3000);

  analogWrite(ENA, 0);//Set the speed of ENA
  analogWrite(ENB, carSpeed);//Set the speed of ENB

  // Left forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(3000);

  // Left backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(3000);

  analogWrite(ENB, 0);//Set the speed of ENB
  delay(3000);
  // forward();
  // delay(1000);  
  // back();
  // delay(1000);
  // left();
  // delay(1000);
  // right();
  // delay(1000);
}