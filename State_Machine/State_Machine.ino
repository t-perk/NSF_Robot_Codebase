/********************************************************************
  IMMERSE Summer Robotics Competition Boilerplate Code
  Autonomous Robot

  The approach of this code is to use an architectured that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.

  Version History
  1.0.0       25 February 2025    Creation by Tyler Perkins with
                                  inspiration from ECEN 240 &
                                  ChatGPT

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include "Arduino.h"
#include <NewPing.h>
#include <PWMServo.h>

const int DebugStateOutput = false; // Change false to true for debug messages

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

// Servo pin
#define SERVO_PIN 10

// Parameters for servo control as well as instantiation
#define SERVO_START_ANGLE 90
#define SERVO_LEFT_LIMIT 180
#define SERVO_RIGHT_LIMIT 0
static PWMServo myServo;

/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Parameters for ultrasonic sensor and instantiation
// Maximum distance we want to ping for (in centimeters). 
#define MAX_DISTANCE 400 

// Parameter to define when the ultrasonic sensor detects a collision - Lab 6
#define STOP_DISTANCE 20

/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Motor speed definitions - Lab 4
#define SPEED_STOP      0
#define SPEED_LOW       (int) (140 + 125 * 0.33)
#define SPEED_MED       (int) (140 + 125 * 0.66)
#define SPEED_HIGH      (int) (140 + 125 * 1)

// Collision definitions
#define COLLISION_OFF 0
#define COLLISION_ON  1

// Driving direction definitions
#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

// Servo movement definitions
#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_LEFT   1
#define SERVO_MOVE_RIGHT 2

/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision;

// IR avoidance state variables
int l_IRAvoidanceSensorState;
int r_IRAvoidanceSensorState;

// Ultrasonic sensor state variable
int UltrasonicDistance;

// Line following IR sensors
int Line_Sensor1;
int Line_Sensor2;
int Line_Sensor3;
int Line_Sensor4;

/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STRAIGHT;
// Speed
// 130 - 255 are generally good on a full battery (on smooth surface)
int ActionRobotSpeed = 120;
int ActionRobotTurnSpeed = 180;
// Servo Action (using Definitions)
int ActionServoMove =  SERVO_MOVE_STOP;

// Flag for stopping the robot
bool StopDebounce = false;
int stopCounter = 0;

// Time units to remain in stop state (approx. in ms)
#define STOP_TIME 220

bool lineFollowingDisabled = false;
int lineFollowingDisabled_Count = 0;

#define LINE_FOLLOWING_DISABLED_TIME 25

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {
  //Set up serial connection at 9600 Baud
  Serial.begin(9600);

  pinMode(IR_AVOID_R, INPUT);
  pinMode(IR_AVOID_R, INPUT);

  // Initialize perception state variables
  SensedCollision = DETECTION_NO;
  l_IRAvoidanceSensorState = 0;
  r_IRAvoidanceSensorState = 0;
  UltrasonicDistance = 0;
  
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
  // myServo.attach(SERVO_PIN);
  // myServo.write(SERVO_START_ANGLE);

  //Set up rangefinder
  pinMode(TRIGGER_PIN, OUTPUT); // pulse sent out through TRIGGER_PIN    
  pinMode(ECHO_PIN, INPUT); // return signal read through ECHO_PIN

  // Setup stop condition variables
  StopDebounce = false;
  stopCounter = 0;

  // Setup line following disabled variables
  lineFollowingDisabled = false;
  lineFollowingDisabled_Count = 0;
}

/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/
void loop() {
  // This DebugStateOutput flag can be used to easily turn on the
  // serial debugging to know what the robot is perceiving and what
  // actions the robot wants to take.

  RobotPerception(); // PERCEPTION
  if (DebugStateOutput) {
    Serial.println("\n------------------------------");
    Serial.println("Perception:");
    
    Serial.print("IRSensorLeft: ");
    Serial.println(l_IRAvoidanceSensorState);
    Serial.print("IRSensorRight: ");
    Serial.println(r_IRAvoidanceSensorState);

    Serial.print("Ultrasonic range: ");
    Serial.println(UltrasonicDistance);

    Serial.print("isCollision state: ");
    Serial.println(SensedCollision);

    Serial.print("IR Line Sensors: ");
    Serial.print(Line_Sensor4);
    Serial.print("--");
    Serial.print(Line_Sensor3);
    Serial.print("--");
    Serial.print(Line_Sensor2);
    Serial.print("--");
    Serial.println(Line_Sensor1);

    Serial.print("StopDebounce value: ");
    Serial.println(StopDebounce);
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.println("\n------------------------------");
    Serial.println("PLANNING");

    Serial.print("ActionRobotDrive: ");
    Serial.println(ActionRobotDrive);

    Serial.print("ActionRobotSpeed: ");
    Serial.println(ActionRobotSpeed);
  }

  RobotAction(); // ACTION

  if (DebugStateOutput){
    delay(1000);
  }
  // delay(1);
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages
  
  l_IRAvoidanceSensorState = digitalRead(IR_AVOID_L);//The sensor on the left
  r_IRAvoidanceSensorState = digitalRead(IR_AVOID_R);//The sensor on the Right

  Line_Sensor1 = digitalRead(LINE_SENSOR_IN1);//IN1
  Line_Sensor2 = digitalRead(LINE_SENSOR_IN2);//IN2
  Line_Sensor3 = digitalRead(LINE_SENSOR_IN3);//IN3
  Line_Sensor4 = digitalRead(LINE_SENSOR_IN4);//IN4

  PingUltrasonicSensor();

  // Collision Sensor
  if (isCollision()) {   // Add code in isCollision() function for lab 2 milestone 1
    SensedCollision = DETECTION_YES;
  } else {
    SensedCollision = DETECTION_NO;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
static NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void PingUltrasonicSensor() {
  // if(sonar.ping_cm() != 0){ // If the distance is too big, it returns 0.
  UltrasonicDistance = sonar.ping_cm();
  // }
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool isCollision() {
  if (UltrasonicDistance < STOP_DISTANCE || l_IRAvoidanceSensorState == 0 || r_IRAvoidanceSensorState == 0) {
    return true;
  } else {
    return false;
  }
}

/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void RobotPlanning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsmCollisionDetection(); // Milestone 1
}

////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsmCollisionDetection() {
  static int collisionDetectionState = 0;
  static int driveState = 3;

  // Handle stop condition
  if (StopDebounce == true){
    if (DebugStateOutput){
      // Serial.println(stopCounter);
    }

    if (stopCounter >= STOP_TIME){
      ActionRobotDrive = DRIVE_STRAIGHT;
      stopCounter = 0;
      StopDebounce = false;
      // Disable line following for a period to ignore the 
      // line immediately below the robot
      lineFollowingDisabled = true;
    } else {
      ActionRobotDrive = DRIVE_STOP;
      stopCounter++;
      return;
    }
  }

  // Driving direction definitions
  // #define DRIVE_STOP      0
  // #define DRIVE_LEFT      1
  // #define DRIVE_RIGHT     2
  // #define DRIVE_STRAIGHT  3
  
  switch (driveState){
    case DRIVE_STRAIGHT:
      ActionRobotDrive = DRIVE_STRAIGHT;
      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        driveState = DRIVE_STRAIGHT; //if no collision, go to no collision state
      } else if (SensedCollision == DETECTION_YES){
        if (!r_IRAvoidanceSensorState){
          driveState = DRIVE_LEFT;
        } else if (!l_IRAvoidanceSensorState){
          driveState = DRIVE_RIGHT;
        }
      }
      break;

    case DRIVE_LEFT:
      ActionRobotDrive = DRIVE_LEFT;

      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        driveState = DRIVE_STRAIGHT; //if no collision, go to no collision state
      } else if (SensedCollision == DETECTION_YES)
      {
        if (!r_IRAvoidanceSensorState){
          driveState = DRIVE_LEFT;
        } else if (!l_IRAvoidanceSensorState){
          if (!r_IRAvoidanceSensorState && (UltrasonicDistance < STOP_DISTANCE)){// If they are both reading a collision, keep r 
            driveState = DRIVE_LEFT;
          } else if (r_IRAvoidanceSensorState && (UltrasonicDistance < STOP_DISTANCE)){
            driveState = DRIVE_RIGHT;
          }
        }
      }
      break;

      case DRIVE_RIGHT:
      ActionRobotDrive = DRIVE_RIGHT;

      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        driveState = DRIVE_STRAIGHT; //if no collision, go to no collision state
      } else if (SensedCollision == DETECTION_YES)
      {
        if (!l_IRAvoidanceSensorState){
          driveState = DRIVE_RIGHT;
        } else if (!r_IRAvoidanceSensorState){
          if (!l_IRAvoidanceSensorState && (UltrasonicDistance < STOP_DISTANCE)){// If they are both reading a collision, keep r 
            driveState = DRIVE_RIGHT;
          } else if (l_IRAvoidanceSensorState && (UltrasonicDistance < STOP_DISTANCE)){
            driveState = DRIVE_LEFT;
          }
        }
      }
      break;

      default: // error handling
      {
        driveState = DRIVE_STOP;
      }
      break;
  }

  //Line following code overwrites the standard obstacle avoidance code UNLESS we are intentionally ignoring it.
  if (lineFollowingDisabled){
    if (lineFollowingDisabled_Count >= LINE_FOLLOWING_DISABLED_TIME){
      lineFollowingDisabled = false;
      lineFollowingDisabled_Count = 0;
      DoLineFollowing();
    }else{
      lineFollowingDisabled_Count++;
    }
  }else{
    //If there's no feedback from the line following sensors, we do nothing
    if (Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
      
    }else{
      DoLineFollowing();
    }
  }
}

void DoLineFollowing(){
  if (DebugStateOutput) {
    Serial.println("Do line following");
  }
  // Line_Sensor1 - Far right
  // Line_Sensor2 - Mid right
  // Line_Sensor3 - Mid left
  // Line_Sensor4 - Far left
  if(Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
    // forward();    
    ActionRobotDrive = DRIVE_STRAIGHT;
  }else if(Line_Sensor4 == HIGH && Line_Sensor3 == HIGH && Line_Sensor2 == HIGH && Line_Sensor1 == HIGH){
    // stop();    
    ActionRobotDrive = DRIVE_STOP;
    StopDebounce = true;
    // Enter state to wait for x seconds before going straight again
  }else if(Line_Sensor4 == HIGH && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
    // left_M();
    ActionRobotDrive = DRIVE_LEFT;
  }else if(Line_Sensor4 == LOW && Line_Sensor3 == HIGH && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
    // left();
    ActionRobotDrive = DRIVE_LEFT;
  }else if(Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == HIGH && Line_Sensor1 == LOW){
    // right();
    ActionRobotDrive = DRIVE_RIGHT;
  }else if(Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == HIGH){
    // right_M();
    ActionRobotDrive = DRIVE_RIGHT;
  }
}

/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void RobotAction() {
  switch(ActionRobotDrive) {
    case DRIVE_STOP:
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, 0);
      break;

    case DRIVE_STRAIGHT:
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);//Set the speed of ENB
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Forward");
      break;

    case DRIVE_RIGHT:
      analogWrite(H_BRIDGE_ENA, ActionRobotTurnSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, ActionRobotTurnSpeed);//Set the speed of ENB
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Right");
      break;

    case DRIVE_LEFT:
      analogWrite(H_BRIDGE_ENA, ActionRobotTurnSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, ActionRobotTurnSpeed);//Set the speed of ENB
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      Serial.println("Left");
      break;
  }
}