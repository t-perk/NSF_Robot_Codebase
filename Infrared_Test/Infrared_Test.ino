// Infrared Reflective Sensor IR Photoelectric Switch
// The digital output is tuned via the onboard potentiometer
// (i.e. the blue box with the screw terminal)

// Define pins
int Sensor1 = A2;//pin A2
int Sensor2 = A5;//pin A5
int SensorLeft;
int SensorRight;

// Setup required pins for the input
void setup() {
  Serial.begin(9600);
  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  delay(1000);
}

// Read sensor state and print to serial monitor
void loop() {
  SensorLeft  =  digitalRead(Sensor1);//The sensor on the left
  SensorRight =  digitalRead(Sensor2);//The sensor on the Right
  Serial.print("SensorLeft: ");
  Serial.println(SensorLeft);
  Serial.print("SensorRight: ");
  Serial.println(SensorRight);
  delay(1000);
}