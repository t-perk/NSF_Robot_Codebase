// Ultrasonic sensor
// See reference code online on Arduino's website

#define TRIGGER_PIN 12
#define ECHO_PIN 13
float cm; // Distance variable

void setup() {
  Serial.begin (9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// Read the data from our ultrasonic sensor
void loop() {
  cm = GetDistance();
  Serial.println(cm);

  delay(500);
}

float GetDistance()
{
    float distance;
    // Send a low short pulse to Trig to trigger the ranging
    digitalWrite(TRIGGER_PIN, LOW); //Send a low level to Trig
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    distance = pulseIn(ECHO_PIN, HIGH) / 58.00;
    Serial.print("Distance = ");
    Serial.println(distance);//The serial output distance is converted into cm
	  return distance;
}