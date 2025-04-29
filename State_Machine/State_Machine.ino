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
                                  inspiration from ECEn 240

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include "Arduino.h"
#include <NewPing.h>
// #include <PWMServo.h>
#include <Servo.h>

const int DebugStateOutput = true; // Change false to true for debug messages

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
#define ARBITRARY_UPDATE_DELAY 10

// Servo pin
#define SERVO_PIN A0

// Parameters for servo control as well as instantiation
#define SERVO_START_ANGLE 90
#define SERVO_LEFT_LIMIT 180//135
#define SERVO_RIGHT_LIMIT 0//45
Servo myServo;

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

/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision;

// IR avoidance state variables
int l_IRAvoidanceSensorState;
int r_IRAvoidanceSensorState;

// Ultrasonic sensor state variables
int straightUltrasonicDistance;
int leftUltrasonicDistance;
int rightUltrasonicDistance;
int updateUltrasonicSensorCounter = 0;
bool sweepRequest = true;
bool turnedLastStep = false;

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
#define SPEED_STRAIGHT_DEFAULT 140
#define SPEED_TURN_SLOW 180 // Previously 220
#define SPEED_TURN_DEFAULT 220 // Previously 220
#define SPEED_TURN_FAST 240

int ActionRobotSpeed = SPEED_STRAIGHT_DEFAULT; // Default is 120
int ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;

// Flag for stopping the robot
int lineFollowing_StopCounter = 0;
int lineFollowing_ForwardCounter = 0;

#define LINEFOLLOWING_STOP_TIME 100
#define LINEFOLLOWING_FORWARD_TIME 5

bool lineFollowingDisabled = false;
int lineFollowingDisabled_Count = 0;
int lineFollowingState = 0;
#define OFFLINE 0
#define ONLINE 1
#define LOCKED 2
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
  straightUltrasonicDistance = 0;
  leftUltrasonicDistance = 0;
  rightUltrasonicDistance = 0;
  
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
    Serial.println(straightUltrasonicDistance);

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

    Serial.print("LF_State: ");
    Serial.print(lineFollowingState);
    Serial.print(" Stop: ");
    Serial.print(lineFollowing_StopCounter);
    Serial.print(" Foward: ");
    Serial.println(lineFollowing_ForwardCounter);
    
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.println("\n------------------------------");
    Serial.println("PLANNING");

    Serial.print("ActionRobotDrive: ");
    Serial.println(ActionRobotDrive);

    // Serial.print("ActionRobotSpeed: ");
    // Serial.println(ActionRobotSpeed);
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

  // if (updateUltrasonicSensorCounter > ULTRASONIC_UPDATE_COUNT){
  //   UltrasonicSensorSweep();
  //   updateUltrasonicSensorCounter = 0;
  // } else {
  //   updateUltrasonicSensorCounter++;
  //   PingUltrasonicSensor();
  // }

  // If a sweep request occurs, check to see if it has already been performed. If so, set the 
  if (sweepRequest){
    UltrasonicSensorSweep();
    updateUltrasonicSensorCounter = 0;
    sweepRequest = false;
  }else{
    PingUltrasonicSensor();
  }

  Serial.print("UltrasonicSensorCounter: ");
  Serial.println(updateUltrasonicSensorCounter);

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

void UltrasonicSensorSweep(){
  Serial.print("Sound Sweep.");
  myServo.write(SERVO_LEFT_LIMIT);
  delay(700); // Wait for the servo to reach its position
  leftUltrasonicDistance = sonar.ping_cm();
  delay(50);
  Serial.print(" L: ");
  Serial.print(leftUltrasonicDistance);

  myServo.write(SERVO_START_ANGLE);
  delay(700);
  straightUltrasonicDistance = sonar.ping_cm();
  delay(50);
  Serial.print(" S: ");
  Serial.print(straightUltrasonicDistance);  

  myServo.write(SERVO_RIGHT_LIMIT);
  delay(700);
  rightUltrasonicDistance = sonar.ping_cm();
  delay(50);
  Serial.print(" R: ");
  Serial.println(rightUltrasonicDistance);  

  // Return to the front
  myServo.write(SERVO_START_ANGLE);
  delay(700);
}

void PingUltrasonicSensor() {
  myServo.write(SERVO_START_ANGLE);
  straightUltrasonicDistance = sonar.ping_cm();
  Serial.println(straightUltrasonicDistance);  
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool isCollision() {
  if (l_IRAvoidanceSensorState == 0 || r_IRAvoidanceSensorState == 0) {
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

  // Driving direction definitions
  // #define DRIVE_STOP      0
  // #define DRIVE_LEFT      1
  // #define DRIVE_RIGHT     2
  // #define DRIVE_STRAIGHT  3

  //We only want to follow the sound sweep information if there 
  //is not a line currently being detected.
  if (!isLineDetected() && lineFollowingState == OFFLINE && !SensedCollision){
    // Serial.println("############drive state##############");
    if (updateUltrasonicSensorCounter >= ULTRASONIC_UPDATE_COUNT){
      ActionRobotDrive = DRIVE_STOP;
      if (!sweepRequest){
        sweepRequest = true;
      }
      return;
    }else{
      updateDriveState();
      updateUltrasonicSensorCounter++;
    }
  }

  if ((isLineDetected() || lineFollowingState == ONLINE || lineFollowingState == LOCKED) && !SensedCollision){
    if (lineFollowingState == OFFLINE){
      lineFollowingState = ONLINE;
      DoLineFollowing();
    }else if (lineFollowingState == ONLINE){
      DoLineFollowing();

    }else if (lineFollowingState == LOCKED){
      if (lineFollowing_StopCounter <= LINEFOLLOWING_STOP_TIME){
        ActionRobotDrive = DRIVE_STOP;
        lineFollowing_StopCounter++;
        return;
      } else if (lineFollowing_ForwardCounter <= LINEFOLLOWING_FORWARD_TIME){
        Serial.println("FORWARD COUNTER");
        ActionRobotDrive = DRIVE_STRAIGHT;
        lineFollowing_ForwardCounter++;
        return;
      } else {
        lineFollowingState = OFFLINE;
        lineFollowing_ForwardCounter = 0;
        lineFollowing_StopCounter = 0;
        updateUltrasonicSensorCounter = ULTRASONIC_UPDATE_COUNT - ARBITRARY_UPDATE_DELAY;
      }
    }
  }

  if (SensedCollision){
    lineFollowingState = OFFLINE;
    lineFollowing_ForwardCounter = 0;
    lineFollowing_StopCounter = 0;

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
      ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;

      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        driveState = DRIVE_STRAIGHT; //if no collision, go to no collision state
      } else if (SensedCollision == DETECTION_YES)
      {
        if (!r_IRAvoidanceSensorState){
          driveState = DRIVE_LEFT;
        } else if (!l_IRAvoidanceSensorState){
          if (!r_IRAvoidanceSensorState && (straightUltrasonicDistance < STOP_DISTANCE)){// If they are both reading a collision, keep r 
            driveState = DRIVE_LEFT;
          } else if (r_IRAvoidanceSensorState && (straightUltrasonicDistance < STOP_DISTANCE)){
            driveState = DRIVE_RIGHT;
          }
        }
      }
      break;

      case DRIVE_RIGHT:
      ActionRobotDrive = DRIVE_RIGHT;
      ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;

      //State transition logic
      if (SensedCollision == DETECTION_NO) {
        driveState = DRIVE_STRAIGHT; //if no collision, go to no collision state
      } else if (SensedCollision == DETECTION_YES){
        if (!l_IRAvoidanceSensorState){
          driveState = DRIVE_RIGHT;
        } else if (!r_IRAvoidanceSensorState){
          if (!l_IRAvoidanceSensorState && (straightUltrasonicDistance < STOP_DISTANCE)){// If they are both reading a collision, keep r 
            driveState = DRIVE_RIGHT;
          } else if (l_IRAvoidanceSensorState && (straightUltrasonicDistance < STOP_DISTANCE)){
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
  }
}

bool isLineDetected(){
  if (Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
    return false;
  }else{
    return true;
  }
}

void updateDriveState(){
  if (((straightUltrasonicDistance > leftUltrasonicDistance) && (straightUltrasonicDistance > rightUltrasonicDistance)) || turnedLastStep){
    ActionRobotDrive = DRIVE_STRAIGHT;
    turnedLastStep = false;
  }else if((leftUltrasonicDistance > straightUltrasonicDistance) && (leftUltrasonicDistance > rightUltrasonicDistance)){
    ActionRobotDrive = DRIVE_LEFT;
    ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;
    turnedLastStep = true;
  }else if((rightUltrasonicDistance > straightUltrasonicDistance) && (rightUltrasonicDistance > leftUltrasonicDistance)){
    ActionRobotDrive = DRIVE_RIGHT;
    ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;
    turnedLastStep = true;
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
    ActionRobotTurnSpeed = SPEED_TURN_DEFAULT;
    lineFollowingState = LOCKED;

  }else if(Line_Sensor3 == HIGH || Line_Sensor4 == HIGH){
    ActionRobotDrive = DRIVE_LEFT;
    ActionRobotTurnSpeed = SPEED_TURN_SLOW;
    if (Line_Sensor2 == HIGH){
      ActionRobotDrive = DRIVE_STRAIGHT;
      // If two of the opposing sensors are high, then that side outvotes the other side
      if (Line_Sensor1 == HIGH){
        ActionRobotDrive == DRIVE_RIGHT;
        ActionRobotTurnSpeed = SPEED_TURN_SLOW;
      }
    }
    if (Line_Sensor4 == HIGH){
      ActionRobotDrive = DRIVE_LEFT;
      ActionRobotTurnSpeed = SPEED_TURN_SLOW;
    }
  } else if (Line_Sensor2 == HIGH || Line_Sensor1 == HIGH){
    ActionRobotDrive = DRIVE_RIGHT;
    ActionRobotTurnSpeed = SPEED_TURN_SLOW;
    if (Line_Sensor3 == HIGH){
      ActionRobotDrive = DRIVE_STRAIGHT;
      // If two of the opposing sensors are high, then that side outvotes the other side
      if (Line_Sensor4 == HIGH){
        ActionRobotDrive == DRIVE_LEFT;
        ActionRobotTurnSpeed = SPEED_TURN_SLOW;
      }
    }
    if (Line_Sensor1 == HIGH){
      ActionRobotDrive = DRIVE_RIGHT;
      ActionRobotTurnSpeed = SPEED_TURN_SLOW;
    }
  }
  if (Line_Sensor4 == HIGH && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == LOW){
    ActionRobotDrive = DRIVE_LEFT;
    ActionRobotTurnSpeed = SPEED_TURN_FAST;
  }

  if (Line_Sensor4 == LOW && Line_Sensor3 == LOW && Line_Sensor2 == LOW && Line_Sensor1 == HIGH){
    ActionRobotDrive = DRIVE_RIGHT;
    ActionRobotTurnSpeed = SPEED_TURN_FAST;
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
      Serial.println("Stop");
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