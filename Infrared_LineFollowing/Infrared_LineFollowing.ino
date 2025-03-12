// Line following infrared sensor
// These sensors work similarly to the regular infrared 
// sensors. We read out an digital value.

#define SENSOR_1_PIN 8
#define SENSOR_2_PIN 9 
#define SENSOR_3_PIN 10
#define SENSOR_4_PIN 11

int Sensor1;
int Sensor2;
int Sensor3;
int Sensor4;

// Setup pins to read in data
void setup() {
  pinMode(SENSOR_1_PIN, INPUT);
  pinMode(SENSOR_2_PIN, INPUT);
  pinMode(SENSOR_3_PIN, INPUT);
  pinMode(SENSOR_4_PIN, INPUT);
  Serial.begin(9600);
}

// Report data values in the serial monitor
void loop() {
  Sensor1 = digitalRead(SENSOR_1_PIN);//IN1
  Sensor2 = digitalRead(SENSOR_2_PIN);//IN2
  Sensor3 = digitalRead(SENSOR_3_PIN);//IN3
  Sensor4 = digitalRead(SENSOR_4_PIN);//IN4

  Serial.print(Sensor4);
  Serial.print("--");
  Serial.print(Sensor3);
  Serial.print("--");
  Serial.print(Sensor2);
  Serial.print("--");
  Serial.println(Sensor1);
}