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

/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STRAIGHT;
// Speed
// 130 - 255 are generally good on a full battery (on smooth surface)
int ActionRobotSpeed = 130;
// Servo Action (using Definitions)
int ActionServoMove =  SERVO_MOVE_STOP;

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
  
  // //Set up servo
  // myServo.attach(SERVO_PIN);
  // myServo.write(SERVO_START_ANGLE);

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
    Serial.println(UltrasonicDistance);

    Serial.print("isCollision state: ");
    Serial.println(SensedCollision);
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.println("\n------------------------------");
    Serial.println("PLANNING");

    Serial.print("ActionCollision: ");
    Serial.println(ActionCollision);

    Serial.print("ActionRobotDrive: ");
    Serial.println(ActionRobotDrive);

    Serial.print("ActionRobotSpeed: ");
    Serial.println(ActionRobotSpeed);
  }

  RobotAction(); // ACTION

  if (DebugStateOutput){
    delay(1000);
  }
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages
  
  l_IRAvoidanceSensorState = digitalRead(IR_AVOID_L);//The sensor on the left
  r_IRAvoidanceSensorState = digitalRead(IR_AVOID_R);//The sensor on the Right

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

// ////////////////////////////////////////////////////////////////////
// // Function that detects if light is present
// ////////////////////////////////////////////////////////////////////
// bool isLight(int pin) {
//   float light = getPinVoltage(pin);
//   //Serial.println(light); // Use this line to test
//   return (light > PHOTODIODE_LIGHT_THRESHOLD);
// }

// ////////////////////////////////////////////////////////////////////
// // Function to read pin voltage
// ////////////////////////////////////////////////////////////////////
// float getPinVoltage(int pin) {
//   //This function can be used for many different tasks in the labs
//   //Study this line of code to understand what is going on!!
//   //What does analogRead(pin) do?
//   //Why is (float) needed?
//   //Why divide by 1024?
//   //Why multiply by 5?
//   return 5 * (float)analogRead(pin) / 1024;
// }

// ////////////////////////////////////////////////////////////////////
// // Function to determine if a button is pushed or not
// ////////////////////////////////////////////////////////////////////
// bool isButtonPushed(int button_pin) {
//   //This function can be used to determine if a said button is pushed.
//   //Remember that when the voltage is 0, it's only close to zero.
//   //Hint: Call the getPinVoltage function and if that value is greater
//   // than the BUTTON_THRESHOLD variable toward the top of the file, return true.
//   if (getPinVoltage(button_pin) >= BUTTON_THRESHOLD) {
//     return(true);
//   }
//   else {
//     return (false);
//   }

// }

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
  static int driveState = 1;
  //Serial.println(collisionDetectionState); //uncomment for debugging


  //   // Driving direction definitions
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
      } else if (SensedCollision == DETECTION_YES)
      {
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
}

// ////////////////////////////////////////////////////////////////////
// // State machine for cycling through the robot's speeds.
// ////////////////////////////////////////////////////////////////////
// void fsmChangeSpeed() {
//   /*Implement in lab 4*/
//    static int changeSpeedState = 0;

//    switch(changeSpeedState)
//    {
//     case 0: // Speed Level 0
//     //Serial.println("SPEED_STOP");
//       ActionRobotSpeed = SPEED_STOP; 
    
//       //State transition
//       changeSpeedState = 1; // Go to speed low
//     break;

//     case 1: // Speed Level 1
//     //Serial.println("SPEED_LOW");
//       ActionRobotSpeed = SPEED_LOW; 
    
//       //State transition
//       changeSpeedState = 2; // Go to speed med
//     break;

//     case 2: // Speed Level 2
//     //Serial.println("SPEED_MED");
//       ActionRobotSpeed = SPEED_MED; 
    
//       //State transition
//       changeSpeedState = 3; // Go to speed high
//     break;

//     case 3: // Speed Level 3
//     //Serial.println("SPEED_HIGH");
//       // Don’t forget to define ActionRobotSpeed
//       ActionRobotSpeed = SPEED_HIGH; 
    
//       //State transition
//       changeSpeedState = 0; // Go to speed stop
//     break;

//     default:
//     {
//       changeSpeedState = 0;
//     }

//    }

// }

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
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);//Set the speed of ENB
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      Serial.println("Right");
      break;

    case DRIVE_LEFT:
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);//Set the speed of ENA
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);//Set the speed of ENB
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      Serial.println("Left");
      break;
  }
}

// /**********************************************************************************************************
//   AUXILIARY functions that may be useful in performing diagnostics
//  ********************************************************************/
// // Function to turn LED on
// void doTurnLedOn(int led_pin)
// {
//   digitalWrite(led_pin, HIGH);   // Turn on the LED
//   /* Use knowledge from lab 1 to set the led_pin on */
// }

// // Function to turn LED off
// void doTurnLedOff(int led_pin)
// {
//   /* Use knowledge from lab 1 to set the led_pin high */
//   digitalWrite(led_pin, LOW);    // Turn off the LED
// }