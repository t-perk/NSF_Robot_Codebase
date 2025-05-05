// - Continuously scanning UltrasonicSensor
// * Populates a dictionary with keys 0-180 in 20-45 degree increments as it scans
// * Control algorithm changes the motor control dynamically based on the sensor information. Control algorithm?

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include "Arduino.h"
#include <NewPing.h>
#include <Servo.h>

const int debugStateOutput = true; // Change false to true for debug messages

//
// Compiler defines: the compiler replaces each name with its assignment
// (These make your code so much more readable and makes constants easier 
// to modify.)
//

/***********************************************************/
// Hardware pin definitions
// Replace the pin numbers with those you connect to your robot

#define IR_AVOID_L A2
#define IR_AVOID_R A5

// Motor enable pins
#define H_BRIDGE_ENA 5
#define H_BRIDGE_ENB 6

#define IN1 3 // right
#define IN2 4 // right
#define IN3 2 // left
#define IN4 7 // left

#define LINE_SENSOR_IN1 8 // far right
#define LINE_SENSOR_IN2 9 // right
#define LINE_SENSOR_IN3 10 // left
#define LINE_SENSOR_IN4 11 // far left

// Ultrasonic sensor pins
#define TRIGGER_PIN 12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 13  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define ULTRASONIC_UPDATE_COUNT 20

// Servo pin
#define SERVO_PIN A0
Servo myServo;

/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Parameters for ultrasonic sensor and instantiation
// Maximum distance we want to ping for (in centimeters). 
#define MAX_DISTANCE 400 

// Parameter to define when the ultrasonic sensor detects a collision
#define STOP_DISTANCE 20


// Parameters for servo control as well as instantiation
#define SERVO_START_ANGLE 90
#define SERVO_LEFT_LIMIT 180//135
#define SERVO_RIGHT_LIMIT 0//45

// Motor speed definitions
// 120 - 255 are generally good on a full battery
#define SPEED_STOP      0
#define SPEED_STRAIGHT_DEFAULT 120
#define SPEED_TURN_SLOW 180 // Previously 220
#define SPEED_TURN_DEFAULT 220 // Previously 220
#define SPEED_TURN_FAST 240

// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Collision definitions
#define COLLISION_OFF 0
#define COLLISION_ON  1

/***********************************************************/
// Driving direction definitions
#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int sensedCollision;

// IR avoidance state variables
int l_IRAvoidanceSensorState;
int r_IRAvoidanceSensorState;

// Ultrasonic sensor state variables
int straightUltrasonicDistance;
int leftUltrasonicDistance;
int rightUltrasonicDistance;
int updateUltrasonicSensorCounter = 0;
bool sweepRequest = true;
bool forwardToggle = true;
int checkPointMillis = 0;
bool turnedLastStep = false;
int turnedCount = 0;

#define CYCLE_TIME_MILLIS 30

// Line following IR sensors
int lineSensor1;
int lineSensor2;
int lineSensor3;
int lineSensor4;

// Flag for stopping the robot
int lineFollowing_StopCounter = 0;
int lineFollowing_ForwardCounter = 0;

#define LINEFOLLOWING_STOP_COUNT 100
#define LINEFOLLOWING_FORWARD_COUNT 5

bool lineFollowingDisabled = false;
int lineFollowingDisabled_Count = 0;
int lineFollowingState = 0;
int conseqDetections = 0;

// Line following states
#define OFFLINE 0
#define ONLINE 1
#define LOCKED 2

int directionArrKeys[] = {0, 45, 90, 135, 180};
int directionArrValues[] = {0,0,0,0};

/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int actionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int actionRobotDrive = DRIVE_STRAIGHT;

int actionRobotSpeed = SPEED_STRAIGHT_DEFAULT;
int actionRobotTurnSpeed = SPEED_TURN_DEFAULT;

void setup() {
    //Set up serial connection at 9600 Baud
    Serial.begin(9600);
  
    pinMode(IR_AVOID_R, INPUT);
    pinMode(IR_AVOID_R, INPUT);
  
    // Initialize perception state variables
    sensedCollision = DETECTION_NO;
    l_IRAvoidanceSensorState = 0;
    r_IRAvoidanceSensorState = 0;
    straightUltrasonicDistance = 0;
    leftUltrasonicDistance = 0;
    rightUltrasonicDistance = 0;
    checkPointMillis = millis();
    
    // //Set up output pins
    pinMode(H_BRIDGE_ENA, OUTPUT);
    pinMode(H_BRIDGE_ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
  
    pinMode(LINE_SENSOR_IN1, INPUT);
    pinMode(LINE_SENSOR_IN2, INPUT);
    pinMode(LINE_SENSOR_IN3, INPUT);
    pinMode(LINE_SENSOR_IN4, INPUT);
    
    // //Set up servo
    myServo.attach(SERVO_PIN);
    myServo.write(SERVO_START_ANGLE);
  
    //Set up rangefinder
    pinMode(TRIGGER_PIN, OUTPUT); // pulse sent out through TRIGGER_PIN    
    pinMode(ECHO_PIN, INPUT); // return signal read through ECHO_PIN
}

/********************************************************************
Main LOOP function - this gets executed in an infinite loop until
power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
********************************************************************/
void loop() {
    // This debugStateOutput flag can be used to easily turn on the
    // serial debugging to know what the robot is perceiving and what
    // actions the robot wants to take.
    robot_perception(); // PERCEPTION
    if (debugStateOutput) {
        Serial.println("\n------------------------------");
        Serial.println("Perception:");
        
        Serial.print("IRSensorLeft: ");
        Serial.println(l_IRAvoidanceSensorState);
        Serial.print("IRSensorRight: ");
        Serial.println(r_IRAvoidanceSensorState);

        Serial.print("Ultrasonic range: ");
        Serial.println(straightUltrasonicDistance);

        Serial.print("is_collision state: ");
        Serial.println(sensedCollision);

        Serial.print("IR Line Sensors: ");
        Serial.print(lineSensor4);
        Serial.print("--");
        Serial.print(lineSensor3);
        Serial.print("--");
        Serial.print(lineSensor2);
        Serial.print("--");
        Serial.println(lineSensor1);

        Serial.print("LF_State: ");
        Serial.print(lineFollowingState);
        Serial.print(" Stop: ");
        Serial.print(lineFollowing_StopCounter);
        Serial.print(" Foward: ");
        Serial.println(lineFollowing_ForwardCounter);
    }

    // robot_planning(); // PLANNING
    // if (debugStateOutput) {
    //     Serial.println("\n------------------------------");
    //     Serial.println("PLANNING");

    //     Serial.print("actionRobotDrive: ");
    //     Serial.println(actionRobotDrive);
    // }

    // robot_action(); // ACTION

    // if (debugStateOutput){
    //     delay(1000);
    // }
    // delay(10);
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void robot_perception() {
    // This function polls all of the sensors and then assigns sensor outputs
    // that can be used by the robot in subsequent stages

    l_IRAvoidanceSensorState = digitalRead(IR_AVOID_L);//The sensor on the left
    r_IRAvoidanceSensorState = digitalRead(IR_AVOID_R);//The sensor on the Right

    lineSensor1 = digitalRead(LINE_SENSOR_IN1);//IN1
    lineSensor2 = digitalRead(LINE_SENSOR_IN2);//IN2
    lineSensor3 = digitalRead(LINE_SENSOR_IN3);//IN3
    lineSensor4 = digitalRead(LINE_SENSOR_IN4);//IN4

    int ultrasonicDistance = 0;
    // Update the sensors
    for(int i = 0; i < sizeof(directionArrKeys); i++){
        moveSensor(directionArrKeys[i]);
        directionArrValues[i] = ping_ultrasonic_sensor();
    }



    // Collision Sensor
    if (is_collision()) {
        sensedCollision = DETECTION_YES;
    } else {
        sensedCollision = DETECTION_NO;
    }
}

