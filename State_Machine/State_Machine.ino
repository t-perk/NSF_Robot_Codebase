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
#define ULTRASONIC_UPDATE_COUNT 30
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

// Parameter to define when the ultrasonic sensor detects a collision
#define STOP_DISTANCE 20

/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Motor speed definitions
// 120 - 255 are generally good on a full battery
#define SPEED_STOP      0
#define SPEED_STRAIGHT_DEFAULT 120
#define SPEED_TURN_SLOW 180 // Previously 220
#define SPEED_TURN_DEFAULT 220 // Previously 220
#define SPEED_TURN_FAST 240

// Collision definitions
#define COLLISION_OFF 0
#define COLLISION_ON  1


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

/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int actionCollision = COLLISION_OFF;

// Main motors Action (using Definitions
int actionRobotSpeed = SPEED_STRAIGHT_DEFAULT;
int actionRobotTurnSpeed = SPEED_TURN_DEFAULT;

// Driving direction definitions
// #define STATE_STOP      0
// #define STATE_LEFT      1
// #define STATE_RIGHT     2
// #define STATE_STRAIGHT  3
enum state_drive{
  STATE_STOP,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_STRAIGHT
};

static enum state_drive currentDriveState;

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
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

  currentDriveState = STATE_STRAIGHT;
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
  
  robot_planning(); // PLANNING
  if (debugStateOutput) {
    Serial.println("\n------------------------------");
    Serial.println("PLANNING");

    Serial.print("currentDriveState: ");
    Serial.println(currentDriveState);
  }

  robot_action(); // ACTION

  if (debugStateOutput){
    delay(1000);
  }
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

  // If a sweep request == true, perform sweep and reset counter
  if (sweepRequest){
    ultrasonic_sensor_sweep();
    updateUltrasonicSensorCounter = 0;
    sweepRequest = false;
  }else{
    ping_ultrasonic_sensor();
  }

  if (debugStateOutput){
    Serial.print("UltrasonicSensorCounter: ");
    Serial.println(updateUltrasonicSensorCounter);
  }
  // Serial.println()

  // Collision Sensor
  if (is_collision()) {
    sensedCollision = DETECTION_YES;
  } else {
    sensedCollision = DETECTION_NO;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
static NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// ultrasonic_sensor_sweep
// Swivel the servo to gauge which direction has the most room
void ultrasonic_sensor_sweep(){
  myServo.write(SERVO_LEFT_LIMIT);
  delay(700); // Wait for the servo to reach its position
  leftUltrasonicDistance = sonar.ping_cm();
  delay(50);

  if (debugStateOutput){
    Serial.print(" L: ");
    Serial.print(leftUltrasonicDistance);
  }

  myServo.write(SERVO_START_ANGLE);
  delay(700);
  straightUltrasonicDistance = sonar.ping_cm();
  delay(50);

  if (debugStateOutput){
    Serial.print(" S: ");
    Serial.print(straightUltrasonicDistance); 
  }

  myServo.write(SERVO_RIGHT_LIMIT);
  delay(700);
  rightUltrasonicDistance = sonar.ping_cm();
  delay(50);

  if (debugStateOutput){
    Serial.print(" R: ");
    Serial.println(rightUltrasonicDistance);
  }  

  // Return to the front
  myServo.write(SERVO_START_ANGLE);
  delay(700);
}

void ping_ultrasonic_sensor() {
  myServo.write(SERVO_START_ANGLE);
  straightUltrasonicDistance = sonar.ping_cm();
  if (debugStateOutput){
    Serial.println(straightUltrasonicDistance);
  }  
}

////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool is_collision() {
  if (l_IRAvoidanceSensorState == 0 || r_IRAvoidanceSensorState == 0) {
    return true;
  } else {
    return false;
  }
}

/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void robot_planning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsm_collision_detection();
}

////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsm_collision_detection() {
  static int collisionDetectionState = 0;
  // updateUltrasonicSensorCounter++;

  // Driving direction definitions
  // #define STATE_STOP      0
  // #define STATE_LEFT      1
  // #define STATE_RIGHT     2
  // #define STATE_STRAIGHT  3

  if (!is_line_detected()){
    conseqDetections = 0;
  }

  // We only want to follow the sound sweep information if there 
  // is not a line currently being detected.
  if (lineFollowingState == OFFLINE && !sensedCollision){
    if (updateUltrasonicSensorCounter >= ULTRASONIC_UPDATE_COUNT){
      currentDriveState = STATE_STOP;
      if (!sweepRequest){
        sweepRequest = true;
      }
      return;
    }else{
      updateUltrasonicSensorCounter++;
      update_drive_state();
    }
  }
  
  // Line following logic depending on line following state
  if ((is_line_detected() || lineFollowingState == ONLINE || lineFollowingState == LOCKED) && !sensedCollision){
    if (lineFollowingState == OFFLINE){
      if (conseqDetections > 3){
        lineFollowingState = ONLINE;
        conseqDetections = 0;
        do_line_following();
      }else{
        conseqDetections++;
      }
    }else if (lineFollowingState == ONLINE){
      do_line_following();

    }else if (lineFollowingState == LOCKED){
      if (lineFollowing_StopCounter <= LINEFOLLOWING_STOP_COUNT){
        currentDriveState = STATE_STOP;
        lineFollowing_StopCounter++;
        return;
      } else if (lineFollowing_ForwardCounter <= LINEFOLLOWING_FORWARD_COUNT){
        currentDriveState = STATE_STRAIGHT;
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

  //IR Obstacle avoidance state machine
  if (sensedCollision){
    lineFollowingState = OFFLINE;
    lineFollowing_ForwardCounter = 0;
    lineFollowing_StopCounter = 0;

    switch (currentDriveState){
    case STATE_STRAIGHT:
      currentDriveState = STATE_STRAIGHT;
      //State transition logic
      if (sensedCollision == DETECTION_NO) {
        currentDriveState = STATE_STRAIGHT; //if no collision, go to no collision state
      } else if (sensedCollision == DETECTION_YES){
        if (!r_IRAvoidanceSensorState){
          currentDriveState = STATE_LEFT;
        } else if (!l_IRAvoidanceSensorState){
          currentDriveState = STATE_RIGHT;
        }
      }
      break;

    case STATE_LEFT:
      currentDriveState = STATE_LEFT;
      actionRobotTurnSpeed = SPEED_TURN_DEFAULT;

      //State transition logic
      if (sensedCollision == DETECTION_NO) {
        currentDriveState = STATE_STRAIGHT; //if no collision, go to no collision state
      } else if (sensedCollision == DETECTION_YES)
      {
        if (!r_IRAvoidanceSensorState){
          currentDriveState = STATE_LEFT;
        } else if (!l_IRAvoidanceSensorState){
          if (!r_IRAvoidanceSensorState){// If they are both reading a collision, keep r 
            currentDriveState = STATE_LEFT;
          } else if (r_IRAvoidanceSensorState){
            currentDriveState = STATE_RIGHT;
          }
        }
      }
      break;

      case STATE_RIGHT:
      currentDriveState = STATE_RIGHT;
      actionRobotTurnSpeed = SPEED_TURN_DEFAULT;

      //State transition logic
      if (sensedCollision == DETECTION_NO) {
        currentDriveState = STATE_STRAIGHT; //if no collision, go to no collision state
      } else if (sensedCollision == DETECTION_YES){
        if (!l_IRAvoidanceSensorState){
          currentDriveState = STATE_RIGHT;
        } else if (!r_IRAvoidanceSensorState){
          if (!l_IRAvoidanceSensorState){// If they are both reading a collision, keep r 
            currentDriveState = STATE_RIGHT;
          } else if (l_IRAvoidanceSensorState){
            currentDriveState = STATE_LEFT;
          }
        }
      }
      break;

      case STATE_STOP:
      currentDriveState = STATE_STRAIGHT;
      break;

      default: // error handling
      {
        currentDriveState = STATE_STOP;
      }
      break;
    }
  }
}

// is_line_detected()
// Returns whether or not any of the line following IR sensors are detecting anything
bool is_line_detected(){
  if (lineSensor4 == LOW && lineSensor3 == LOW && lineSensor2 == LOW && lineSensor1 == LOW){
    return false;
  }else{
    return true;
  }
}

// update_drive_state()
// update drive state dpending on perception information
void update_drive_state(){
  int currentMillis = millis();

  if (((straightUltrasonicDistance > leftUltrasonicDistance) && (straightUltrasonicDistance > rightUltrasonicDistance))){
    currentDriveState = STATE_STRAIGHT;
    actionRobotSpeed = SPEED_STRAIGHT_DEFAULT;

  }else if((leftUltrasonicDistance > straightUltrasonicDistance) && (leftUltrasonicDistance > rightUltrasonicDistance)){
    if (forwardToggle){
      if ((currentMillis - checkPointMillis) <= CYCLE_TIME_MILLIS){
        currentDriveState = STATE_STRAIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;    
      }else{
        checkPointMillis = millis();
        forwardToggle = !forwardToggle;
        currentDriveState = STATE_LEFT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;
      }
    }else{
      if ((currentMillis - checkPointMillis) <= CYCLE_TIME_MILLIS){
        currentDriveState = STATE_LEFT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;    
      }else{
        checkPointMillis = millis();
        forwardToggle = !forwardToggle;
        currentDriveState = STATE_STRAIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;
      }
    }

  } else if((rightUltrasonicDistance > straightUltrasonicDistance) && (rightUltrasonicDistance > leftUltrasonicDistance)){
    if (forwardToggle){
      if ((currentMillis - checkPointMillis) <= CYCLE_TIME_MILLIS){
        currentDriveState = STATE_STRAIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;    
      }else{
        checkPointMillis = millis();
        forwardToggle = !forwardToggle;
        currentDriveState = STATE_RIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;
      }
    }else{
      if ((currentMillis - checkPointMillis) <= CYCLE_TIME_MILLIS){
        currentDriveState = STATE_RIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;    
      }else{
        checkPointMillis = millis();
        forwardToggle = !forwardToggle;
        currentDriveState = STATE_STRAIGHT;
        actionRobotTurnSpeed = SPEED_TURN_DEFAULT;
      }
    }
  }
}

// Line following control based on perception information
void do_line_following(){
  if (debugStateOutput) {
    Serial.println("Do line following");
  }
  // lineSensor1 - Far right
  // lineSensor2 - Mid right
  // lineSensor3 - Mid left
  // lineSensor4 - Far left
  if(lineSensor4 == LOW && lineSensor3 == LOW && lineSensor2 == LOW && lineSensor1 == LOW){
    // forward();    
    currentDriveState = STATE_STRAIGHT;
    // This is where you'd add the timeout
  }else if(lineSensor4 == HIGH && lineSensor3 == HIGH && lineSensor2 == HIGH && lineSensor1 == HIGH){
    // stop();    
    currentDriveState = STATE_STOP;
    actionRobotTurnSpeed = SPEED_TURN_DEFAULT;
    lineFollowingState = LOCKED;

  }else if(lineSensor3 == HIGH || lineSensor4 == HIGH){
    currentDriveState = STATE_LEFT;
    actionRobotTurnSpeed = SPEED_TURN_SLOW;
    if (lineSensor2 == HIGH){
      currentDriveState = STATE_STRAIGHT;
      // If two of the opposing sensors are high, then that side outvotes the other side
      if (lineSensor1 == HIGH){
        currentDriveState == STATE_RIGHT;
        actionRobotTurnSpeed = SPEED_TURN_SLOW;
      }
    }
    if (lineSensor4 == HIGH){
      currentDriveState = STATE_LEFT;
      actionRobotTurnSpeed = SPEED_TURN_SLOW;
    }
  } else if (lineSensor2 == HIGH || lineSensor1 == HIGH){
    currentDriveState = STATE_RIGHT;
    actionRobotTurnSpeed = SPEED_TURN_SLOW;
    if (lineSensor3 == HIGH){
      currentDriveState = STATE_STRAIGHT;
      // If two of the opposing sensors are high, then that side outvotes the other side
      if (lineSensor4 == HIGH){
        currentDriveState == STATE_LEFT;
        actionRobotTurnSpeed = SPEED_TURN_SLOW;
      }
    }
    if (lineSensor1 == HIGH){
      currentDriveState = STATE_RIGHT;
      actionRobotTurnSpeed = SPEED_TURN_SLOW;
    }
  }
  if (lineSensor4 == HIGH && lineSensor3 == LOW && lineSensor2 == LOW && lineSensor1 == LOW){
    currentDriveState = STATE_LEFT;
    actionRobotTurnSpeed = SPEED_TURN_FAST;
  }

  if (lineSensor4 == LOW && lineSensor3 == LOW && lineSensor2 == LOW && lineSensor1 == HIGH){
    currentDriveState = STATE_RIGHT;
    actionRobotTurnSpeed = SPEED_TURN_FAST;
  }
}

/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void robot_action() {
  switch(currentDriveState) {
    case STATE_STOP:
      analogWrite(H_BRIDGE_ENA, 0);
      analogWrite(H_BRIDGE_ENB, 0);
      Serial.println("Stop");
      break;

    case STATE_STRAIGHT:
      analogWrite(H_BRIDGE_ENA, actionRobotSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, actionRobotSpeed);//Set the speed of ENB
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Forward");
      break;

    case STATE_RIGHT:
      analogWrite(H_BRIDGE_ENA, actionRobotTurnSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, actionRobotTurnSpeed);//Set the speed of ENB
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Right");
      break;

    case STATE_LEFT:
      analogWrite(H_BRIDGE_ENA, actionRobotTurnSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, actionRobotTurnSpeed);//Set the speed of ENB
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      Serial.println("Left");
      break;
  }
}