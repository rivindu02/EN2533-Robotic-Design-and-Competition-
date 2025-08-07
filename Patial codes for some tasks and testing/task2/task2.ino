#include <L298N.h>
#include <Servo.h>
#include <string.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h" 
 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);




// A====> left
// B====> Right


#define PWMA 4
#define PWMB 9
#define AIN1 5
#define AIN2 6
#define BIN1 7
#define BIN2 8


// for IR_array
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
int IR_weight[SensorCount] = {-50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];





// PID control parameters
float Kp = 5 ; // Proportional term
float Ki = 0; // Integral term
float Kd = 5;// Derivative term
float sum;
int increment_count = 0;

long ldistance;
long rdistance;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 115;

String junction = "a";


// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft();
void turnRight();
void noLine();
String colordetect();

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

#define ENCODER_A1 2       // Encoder A channel (Interrupt pin)
#define ENCODER_B1 3     // Encoder B channel (Interrupt pin)

// Encoder counts
volatile long encoderCountA = 0;  // Encoder count for Motor A
volatile long encoderCountB = 0;  // Encoder count for Motor B
const long targetCounts = 190;            // Desired encoder counts for 90 degree turns
const long targetCounts_rotation = 360;   // Desired encoder counts for 180 degree turns


// PID parameters for encoders
//float Kpenco = 9;  // Proportional gain
//float Kienco = 0;  // Integral gain
//float Kdenco = 0;  // Derivative gain

// PID variables
float errorenco = 0;
float previousErrorenco = 0;
float integralenco = 0;
float derivativeenco = 0;

// Target speed for motors
int baseSpeed = 130;  // Base speed for both motors (0 to 255)///////// for Line Navigation 115

int modulus=0;    // for experiment ONLY

const int size_1b=3;
const String Turn_1b[size_1b]={"RT","TP","TT"};
const String Actions_1b[size_1b]={"180","L","B"};
const String LED_1b[size_1b]={"N","N","S"};


const int size_1r=11;
const String Turn_1r[size_1r]={"RT","TP","LL","LT","LT","LT","LT","LT","TT","TP","TT"};
const String Actions_1r[size_1r]={"180","F","L","L","F","F","B1","L","R","R","B"};
const String LED_1r[size_1r]={"N","N","N","N","S","N","O","N","N","N","S"};

const int size_2b=9;
const String Turn_2b[size_2b]={"RT","TP","TT","LT","LT","LT","TT","TP","TT"};
const String Actions_2b[size_2b]={"F","R","R","F","B1","R","L","L","B"};
const String LED_2b[size_2b]={"N","N","N","S","O","N","N","N","S"};

const int size_2r=9;
const String Turn_2r[size_2r]={"RT","TP","TT","LT","LT","LT","TT","TP","TT"};
const String Actions_2r[size_2r]={"180","L","L","F","B1","L","R","R","B"};
const String LED_2r[size_2r]={"N","N","N","S","O","N","N","N","S"};

// Actions
// B1----> backward 1 line----> Array should pass the junction and stop delay 500
// B2----> backward 2 line
// B4----> backward 4 line
// S-----> stop for look for the wall

int Array_size;






void setup() {

  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(50, OUTPUT);        //  led indicate in virtual box 

  // Encoder pins setup
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);


  //half_rotation();
  motor1.stop();
  motor2.stop();

  // give pinmode to defined leds

  //give initial values to servo motors

  
  digitalWrite(53,LOW);
  digitalWrite(51,LOW);

  //backward_B3();
  Serial.begin(9600);

}

void loop() {
  //move();


 // Call the PID function to keep the robot moving straight
 /*moveStraightPID();

  // Add a small delay to control the PID loop frequency
 delay(10);*/

 //task1();

  //VirtualBox(1);
  //encoder_backward();
  
  VirtualBox_new(2);
  //Identify_action("180");
  //find_path_full_array();

  //find_path();
  // //VirtualBox_new(1);
  //Identify_action("B");


 
}
void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 80 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
  }
}

void line_follow(int *sensorValues) {
  
  if (isImmediateTurnL(sensorValues)) {
    //junction = "L";
    motor1.stop();
    motor2.stop();
    delay(5);   // 500
    
    //digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    
      
  }
  else if (isImmediateTurnR(sensorValues)) {
    //junction = "R";
    motor1.stop();
    motor2.stop();
    delay(5);   //500
    //digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    //moveForward(sensorValues);
  }
  else if (isImmediateTurnT(sensorValues)) {
    //junction = "T";
    motor1.stop();
    motor2.stop();
    delay(5);     // 500
    //digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    //moveForward(sensorValues);
  }

  else{
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    //digitalWrite(26, LOW);
    //digitalWrite(28, HIGH);
    //digitalWrite(30, LOW);
  }
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  // Loop through all sensors
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) { // Assuming 1 indicates line detected
      position += IR_weight[(i)];
      onLine++;
    }
  }

  // If no line is detected by any sensor, use the last known error value
  if (onLine == 0) {
    // If previous error is not available, assume the line is straight ahead
    error = -previousError;
  }
  else {
    // Calculate the average position of the line
    position /= onLine;
    // Calculate error based on sensor position
    error = 0 - position;
  }
  for (int i = 49; i > 0; i--) {
    errorArray[i] = errorArray[i - 1];
  }
  errorArray[0] = error;
  // PID terms
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  // Calculate PID value
  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);


  return pidValue;
}

void PID_Linefollow(float pidValue) {
  lsp = lfspeed - pidValue;
  rsp = lfspeed + pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = 255;
  }
  motor_drive(lsp, rsp);
}

void motor_drive(float left, float right) {
  int absRight = abs(right); // Absolute value for right speed
  int absLeft = abs(left);   // Absolute value for left speed

  if (left > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }

  if (right > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();                                             
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }
}

bool isImmediateTurnL(int *sensorValues) {
 
  bool turnLeft = (sensorValues[0] == 0 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) || (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1);
  return (turnLeft);
}
bool isImmediateTurnR(int *sensorValues) {
  bool turnRight = (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 0) || (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0) ; 
  return (turnRight);
}
bool isImmediateTurnT(int *sensorValues) {
  bool turnT = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1; 
  return (turnT);
}

void againcheck(int *sensorValues){
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 15 && encoderCountB < 15 ) {
    
    moveStraightPID();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(5);     // 1000
  readSensors(sensorValues);
  if (isImmediateTurnL(sensorValues)) {
    junction = "L";
    motor1.stop();
    motor2.stop();
    delay(5);     // 500
    moveForward(sensorValues);

  }
  else if (isImmediateTurnR(sensorValues)) {
    junction = "R";
    motor1.stop();
    motor2.stop();
    delay(5);   // 500
    //digitalWrite(26, HIGH);
    //delay(2000);
    moveForward(sensorValues);
  }
  else if (isImmediateTurnT(sensorValues)) {
    junction = "T";
    motor1.stop();
    motor2.stop();
    delay(5);   // 500
    //digitalWrite(26, HIGH);
    //delay(20)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              00);
    moveForward(sensorValues);
  }

  else{
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    //digitalWrite(26, LOW);
    //digitalWrite(28, HIGH);
    //digitalWrite(30, LOW);
  }

}


void backward_B1() {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 450 && encoderCountB < 450 ) {
    
    encoder_backward();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000); // 4000
}

void backward_B3() {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 1090 && encoderCountB < 1090) {
    
    encoder_backward();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000); // 4000
}

void backward_B2() {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 765 && encoderCountB < 765 ) {
    
    encoder_backward();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000); // 4000
}




void moveForward(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  //Serial.print("First");
  //Serial.println(junction);
  readSensors(sensorValues);
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 85 && encoderCountB < 85 ) {
    
    moveStraightPID();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(50);    //1000
  readSensors(sensorValues);
  
  if (junction=="L"){
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2){
      junction="LL";
      //Serial.println(junction);
      
    }
    else if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6){
      junction="TB";
      //Serial.println(junction);
    }
    else{
      junction="LT";
      //Serial.println(junction);
    }

  }
  if (junction=="R"){
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2){
      junction="RR";
      //Serial.println(junction);
      
    }
    else if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6){
      junction="TB";
      //Serial.println(junction);
    }
    else{
      junction="RT";
      //Serial.println(junction);
    }

  }
  if (junction=="T"){
    if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 2){
      junction="TT";
      //Serial.println(junction);
      
    }
    else if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] > 6 ){
      junction="TB";
      //Serial.println(junction);
    }
    else{
      junction="TP";
      //Serial.println(junction);
    }


  }

}



/*void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  readSensors(sensorValues);
  
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1  && sensorValues[6] == 1 && sensorValues[7] == 1 ){
    encoderCountA = 0;
    encoderCountB = 0;

    while (encoderCountA < 75) {
      motor1.setSpeed(95);
      motor1.forward();
      motor2.setSpeed(115);
      motor2.forward();
    }
    motor1.stop();
    motor2.stop();

  }
  
  
  else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 0) {
    turnRight();
    Serial.println("Hiiii");

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnLeft();
    Serial.println("Hellooo");
  }
  else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}
*/
void turnRight() {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while ((encoderCountA < targetCounts) && (encoderCountB < targetCounts) ) {
    
    moveTurnRightPID();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB);
}

void turnLeft() {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountB < targetCounts && encoderCountA < targetCounts) {
    //Serial.println(encoderCountA,encoderCountB);
 
    moveTurnLeftPID();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB);
}


void encoderISR_A() {
  encoderCountA++;
}

// Interrupt Service Routine (ISR) for Motor B encoder
void encoderISR_B() {
  encoderCountB++;
}

void move(){      // testing

  readSensors(sensorValues);
  line_follow(sensorValues);

  if (junction=="LL"){
    //Serial.println(junction);
    turnLeft();
  }
  else if(junction=="LT"){
    //Serial.println(junction);
    //line_follow(sensorValues);
    turnLeft();
  }
  else if(junction=="RR"){
    //Serial.println(junction);
    turnRight();
  }
  else if(junction=="RT"){
    //Serial.println(junction);
    turnRight();
  }
  else if(junction=="TP"){
    //Serial.println(junction);
    line_follow(sensorValues);
    //turnRight();
  }
  else if(junction=="TT"){
    //Serial.println(junction);
    turnRight();
    //turnLeft();
  }
  else if(junction=="TB"){
    //Serial.println(junction);
    motor1.stop();
    motor2.stop();
    while (1);
  }
  junction="a";
}

void moveStraightPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.forward();
  motor2.setSpeed(motor2Speed);
  motor2.forward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}

void VirtualBox_new(int modulus){
  if (modulus==0){
    String Path[8]={"RR","RT","RR","RT","LT","LT","TT","TP"};
    String Actions[8]={"R","F","B1","R","F","L","L","S"};
    String LED[8]={"N","S","O","N","N","N","N","N"};
    Array_size=8;
    find_path_full_array(Path,Actions,LED);

    bool wall_there=false;     //red case
    if (wall_there){
      String Path_[14]={"LL","TP","LT","TT","RT","TT","RR","TP","RT","TP","RT","TT","LT","TP"};
      // FF-----> 
      String Actions_[14]={"B1","B1","L","R","F","R","R","F","F","B1","R","L","L","FF"};
      String LED_[14]={"S","O","N","N","N","N","N","S","S","O","N","N","N","S"};
      Array_size=14;
      find_path_full_array(Path_,Actions_,LED_);
    }
    else{
      String Path_[6]={"LL","TP","LT","TT","RT","TP"};
      String Actions_[6]={"B1","B1","L","R","R","FF"};
      String LED_[6]={"S","O","N","N","N","S"};
      Array_size=6;
      find_path_full_array(Path_,Actions_,LED_);
    }

  }
  else{
    String Path[4]={"RR","RT","RR","TP"};
    String Actions[4]={"R","F","R","S"};
    Array_size=4;
    find_path(Path,Actions);
    // check blue wall
    bool wall_there=true;     // blue case
    if (wall_there){
      switch (modulus){
        case 1:
          Array_size=size_1r;
          find_path_full_array(Turn_1r,Actions_1r,LED_1r);
          break;
        case 2:
          Array_size=size_2r;
          find_path_full_array(Turn_2r,Actions_2r,LED_2r);
          break;
        case 3:
          Array_size=size_1r;
          find_path_full_array(Turn_1r,Actions_1r,LED_1r);
          break;
        case 4:
          Array_size=size_1r;
          find_path_full_array(Turn_1r,Actions_1r,LED_1r);
          break;

      }
      

    }else{
      switch (modulus){
        case 1:
          Array_size=size_1b;
          find_path_full_array(Turn_1b,Actions_1b,LED_1b);
          break;
        case 2:
          Array_size=size_2b;
          find_path_full_array(Turn_2b,Actions_2b,LED_2b);
          break;
        case 3:
          Array_size=size_1b;
          find_path_full_array(Turn_1b,Actions_1b,LED_1b);
          break;
        case 4:
          Array_size=size_1b;
          find_path_full_array(Turn_1b,Actions_1b,LED_1b);
          break;
      }
    }
  }
  

}











void VirtualBox(int modulus){
  
  while(true){
    readSensors(sensorValues);
    line_follow(sensorValues);
    
    if(junction=="RR"){
      turnRight();
    }
    if (junction=="RT"){
      junction="a";         // new modification
      if (modulus==0){

      }else{                        // for modulus=1,2,3,4
        while(true){
          readSensors(sensorValues);
          line_follow(sensorValues);
          if (junction=="RR"){
            turnRight();
          }

          if (junction=="TP"){
            int Array_size =size_1b;
            motor1.stop();
            motor2.stop();
            delay(5000);
            //wall check********************************************

            find_path_full_array(Turn_1b,Actions_1b,LED_1b);


            //while(true);       // check blue box gate
            //  Assume that blue Check_Blue_Wall gives false
            // if (!false ){
            //   switch (modulus) {
            //     case 1:
            //       //char Turn=Turn_1r[10];
            //       //char Action=Actions_1r[10];
            //       //char LED=LED_1r[10];
                  
            //       break;
            //     case 2:
            //       break;
            //   }
            // }else{
            //   switch (modulus) {
            //     case 1:
            //       //char Turn=Turn_1b[2];
            //       //char Action=Actions_1b[2];
            //       //char LED=LED_1b[2];
            //       //find_path_full_array(Turn_1b,Actions_1b,LED_1b,size_1b);
            //       break;
            //     case 2:
            //       break;
            //   }
            // }
          }

          junction="a";
        }
      }  
    }
    junction="a";
  }
    

}



// D53-blue , D51
String colordetect(){
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (b > r){
    digitalWrite(53,HIGH);
    digitalWrite(51,LOW);
    return "Blue" ;
  } 
  if (b < r){
    digitalWrite(51,HIGH);
    digitalWrite(53,LOW);
    return "Red";
  }
  else{
    return "non";
  }
}
bool Blue_Red_NOTdetect(){
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (40<b && b<100){ //(26<r && 26<b && r<200 && b<200){
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    return false ;
  } else{
    return true;
  }


}



void colordetect1(){
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (b > r){
    digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    Serial.print("Red    :");     //32---- 37
    Serial.print(r);
    Serial.print("   Blue    :");    //59--- 64
    Serial.println(b);
  } 
  if (b < r){
    //digitalWrite(51,HIGH);
    //digitalWrite(53,LOW);
    Serial.print("Red    :");     //102
    Serial.print(r);
    Serial.print("   Blue    :");   //50
    Serial.println(b);
  }
}
// Black-----> r=20   b=18
// White-----> r=310  b=360


// const int size_1b=2;
// const String Turn_1b[size_1b]={"TP","TT"};
// const String Actions_1b[size_1b]={"R","B"};
// const String LED_1b[size_1b]={"N","S"};


// const int size_1r=10;
// const String Turn_1r[size_1r]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
// const String Actions_1r[size_1r]={"180","L","L","F","F","180","R","R","R","B"};
// const String LED_1r[size_1r]={"N","N","N","S","N","O","N","N","N","S"};


void Identify_action(String Action){

  if (Action=="180"){half_rotation();}

  if (Action=="L"){turnLeft();}

  if (Action=="R"){turnRight();}

  if (Action=="B"){
    backward_B1();
    while(Blue_Red_NOTdetect()){
      //colordetect1();
      encoder_backward();
    }
    motor1.stop();
    motor2.stop();
    delay(3000);
    
  }
    if (Action=="FF"){
    while(Blue_Red_NOTdetect()){
      //colordetect1();
      moveStraightPID();
    }
    motor1.stop();
    motor2.stop();
    delay(3000);
    
  }


  if (Action=="S"){
    motor1.stop();
    motor2.stop();
    delay(3000);
    }

  if (Action=="F"){line_follow(sensorValues);}

  if (Action=="B1"){backward_B1();}

  if (Action=="B2"){backward_B2();}

  if (Action=="B3"){backward_B3();}


}

void half_rotation() {
  // Perform a sharp left turn by 
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while ((encoderCountB < targetCounts_rotation) && (encoderCountA < targetCounts_rotation)) {
    //Serial.println(encoderCountA,encoderCountB);

    moveTurnRightPID();
 
    // motor2.setSpeed(120);
    // motor2.forward();
    // motor1.setSpeed(120);
    // motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB);
}


bool Identify_Strip(int* sensorValues){
  readSensors(sensorValues);
  if ((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7])>6){
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 1;     
    
  }else{
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 0;
  }
}






// Robo should go backward until the box should increment the increment count and go to next task
void encoder_backward(){

    // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.backward();
  motor2.setSpeed(motor2Speed);
  motor2.backward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}     







void LED_Switching_Juc(String LEDstate){
  // S---> Switch On
  // O---> Switch Off
  // N---> Nothing
  if (LEDstate=="S"){
    // Turn on LED
    digitalWrite(50,HIGH);
  }
  if (LEDstate=="O"){
    // Turn Off LED
    digitalWrite(50,LOW);
  }
}

// const int size_1b=2;
// const String Turn_1b[size_1b]={"TP","TT"};
// const String Actions_1b[size_1b]={"R","B"};
// const String LED_1b[size_1b]={"N","S"};


// const int size_1r=10;
// const String Turn_1r[size_1r]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
// const String Actions_1r[size_1r]={"180","L","L","F","F","180","R","R","R","B"};
// const String LED_1r[size_1r]={"N","N","N","S","N","O","N","N","N","S"};

void find_path(String Turn[], String Actions[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        Identify_action(Actions[i]);
        junction="a";
        break;
      }
      
    }
  }
}



void find_path_full_array(String Turn[], String Actions[],String LED[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        LED_Switching_Juc(LED[i]);
        Identify_action(Actions[i]);
        junction="a";
        break;
      }
      
    }
  }
}

// for check Blue wall open or not
//bool Check_Blue_Wall(int )        <======== have to find out how Ultra sonic works

String Colour;

String Arranging_order(String Colour){
  if (Colour=="Blue"){return "Ascending";}      // Ascending order
  if (Colour=="Red"){return "Descending";}
}

//  Box_Detection function should feed Ultra Sound input



bool Box_Detection(){

  return 0;

}


void Box_lift(){}

void Box_drop(){}


void Box_Arrange(){
  Colour="Blue";
  if (Arranging_order(Colour)="Ascending"){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);

      if (junction=="TP"){
        turnLeft();
      }
      
      junction="a";

      if (Box_Detection()){
        motor1.stop();
        motor2.stop();
        if  (Hight_Measure()==1){
          delay(7000);
          Box_lift();
          half_rotation();


          // box=3, location=A
          const String Turn_3A[3]={"TP","TP","TP"};
          const String Action_3A[3]={"L","F","R"};

          //find_path_Box_Arrangement(Turn_3A,Action_3A);
          Box_drop();

          while(true){
            readSensors(sensorValues);
            line_follow(sensorValues);

            if  (junction=="TP"){
              turnLeft();

              while(true){
                readSensors(sensorValues);
                line_follow(sensorValues);

                if  (junction=="TP"){
                  turnRight();

                }

                if (Box_Detection()){
                  motor1.stop();
                  motor2.stop();
                  if  (Hight_Measure()==1){
                    delay(7000);
                    Box_lift();
                    half_rotation();

                    //const String Turn_1B[2]={"TP","TP"};
                    //const String Turn_1B[2]={"R","L"};

                    //find_path_Box_Arrangement(Turn_1B,Action_1B);
                    Box_drop();
                    while (true){
                      readSensors(sensorValues);
                      line_follow(sensorValues);
                      
                      junction="a";
                    }
          
        
                  }

                }
              junction="a";
              }
            }

            junction="a";

          }
        }

        
        

      }


    
  }

  }
  if (Arranging_order(Colour)="Descending"){


  }
}

void Mech_Arm(String Drop){

}

int Hight_Measure(){
  // there are 3 different angles for hight measurements 
  bool range1;//1
  bool range2;//2
  bool range3;//3
  if (range1){
    return 1;
  }
    if (range2){
    return 2;
  }
    if (range3){
    return 3;
  }
}


//String Box_Arrangement_Turns[];


void find_path_Box_Arrangement(String Turn[], String Actions[]){


  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        Identify_action(Actions[i]);
        break;
        //Mech_Arm(Drop[i]);
      }
      if (junction=="TB"){        // white Box*********** DON't INVERT
        break;

      }
      junction="a";
    }
  }

}


void moveTurnRightPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.forward();
  motor2.setSpeed(motor2Speed);
  motor2.backward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}

void moveTurnLeftPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA - encoderCountB;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kp * errorenco) + (Ki * integralenco) + (Kd * derivativeenco);

  // Adjust motor speeds based on the correction
  int motor1Speed = baseSpeed - correction;  // Motor A
  int motor2Speed = baseSpeed + correction;  // Motor B

  // Constrain the motor speeds to valid PWM values (0 to 255)
  motor1Speed = constrain(motor1Speed, 0, 255);
  motor2Speed = constrain(motor2Speed, 0, 255);
  //Serial.println(motor1Speed);
  //Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.backward();
  motor2.setSpeed(motor2Speed);
  motor2.forward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;

}
