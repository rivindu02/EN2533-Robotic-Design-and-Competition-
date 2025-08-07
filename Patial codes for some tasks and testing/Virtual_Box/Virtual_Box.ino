#include <L298N.h>
#include <Servo.h>
#include <string.h>


#define PWMA 2
#define PWMB 7
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6


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
int lfspeed = 95;

String junction = "a";


// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

#define ENCODER_A1 18       // Encoder A channel (Interrupt pin)
#define ENCODER_B1 20     // Encoder B channel (Interrupt pin)

// Encoder counts
volatile long encoderCountA = 0;  // Encoder count for Motor A
volatile long encoderCountB = 0;  // Encoder count for Motor B
const long targetCounts = 190;   // Desired encoder counts for forward movement


// For Virtual box ----> After getting modulus from LineNavigation

int modulus=1;    // for experiment ONLY

char* Turn_1b[2]={"TP","TT"};
char* Actions_1b[2]={"R","B"};
char* LED_1b[2]={"N","S"};
int size_1b=2;

char* Turn_1r[10]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
char* Actions_1r[10]={"180","L","L","F","F","180","R","R","R","B"};
char* LED_1r[10]={"N","N","N","S","N","S","N","N","N","S"};
int size_1r=10;



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

  // Encoder pins setup
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);



  motor1.stop();
  motor2.stop();

  // give pinmode to defined leds

  //give initial values to servo motors

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  


  //ColourLineFollow();
  
  VirtualBox(modulus);
  
  

}
void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
  }
}

void ColourLineFollow(){
  while (true){
    readSensors(sensorValues);
    line_follow(sensorValues);
    if (junction=="LL"){
      Serial.println(junction);
      turnLeft(sensorValues);
    }
    else if(junction=="LT"){
      Serial.println(junction);
      //line_follow(sensorValues);
      turnLeft(sensorValues);
    }
    else if(junction=="RR"){
      Serial.println(junction);
      turnRight(sensorValues);
    }
    else if(junction=="TP"){
      Serial.println(junction);
      line_follow(sensorValues);
    }
    else if(junction=="TT"){
      Serial.println(junction);
      turnRight(sensorValues);
      //turnLeft(sensorValues);
    }
    else if(junction=="TB"){
      Serial.println(junction);
      motor1.stop();
      motor2.stop();
      while (1);
    }

  junction="a";
  }
  

}


void VirtualBox(int modulus){
  
  while(true){
    readSensors(sensorValues);
    line_follow(sensorValues);
    
    if(junction=="RR"){
      turnRight(sensorValues);
    }
    if (junction=="RT"){
      if (modulus==0){

      }else{                        // for modulus=1,2,3,4
        while(true){
          readSensors(sensorValues);
          line_follow(sensorValues);
          if (junction=="RR"){
            turnRight(sensorValues);
          }

          if (junction=="TP"){
            motor1.stop();
            motor2.stop();
            delay(400);       // check blue box gate
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
            //       //find_path(Turn_1b,Actions_1b,LED_1b,size_1b);
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

//{"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};

void find_path(String Turn, String Action, String LED, int size){
  for (int i=0; i<size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      //if (passed_turn==junction){

      //}
      junction="a";
    }
  }



}

// for check Blue wall open or not
//bool Check_Blue_Wall(int )        <======== have to find out how Ultra sonic works




void line_follow(int *sensorValues) {
  
  if (isImmediateTurnL(sensorValues)) {
    junction = "L";
    motor1.stop();
    motor2.stop();
    delay(500);
    digitalWrite(26, HIGH);
    //delay(2000);
    moveForward(sensorValues);
    
  }
  else if (isImmediateTurnR(sensorValues)) {
    junction = "R";
    motor1.stop();
    motor2.stop();
    delay(500);
    digitalWrite(26, HIGH);
    //delay(2000);
    moveForward(sensorValues);
  }
  else if (isImmediateTurnT(sensorValues)) {
    junction = "T";
    motor1.stop();
    motor2.stop();
    delay(500);
    digitalWrite(26, HIGH);
    //delay(2000);
    moveForward(sensorValues);
  }

  else{
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    digitalWrite(26, LOW);
    digitalWrite(28, HIGH);
    digitalWrite(30, LOW);
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
void moveForward(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  Serial.println(junction);
  readSensors(sensorValues);
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 100 && encoderCountB < 100 ) {
    
    motor1.setSpeed(95);
    motor1.forward();
    motor2.setSpeed(115);
    motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);                // why this delay
  readSensors(sensorValues);
  
  if (junction=="L"){
    if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
      junction="LL";
      
    }
    else if(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){
      junction="TB";
    }
    else{
      junction="LT";
    }

  }
  if (junction=="R"){
    if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
      junction="RR";
      
    }
    else if(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){
      junction="TB";
    }
    else{
      junction="RT";
    }

  }
  if (junction=="T"){
    if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0){
      junction="TT";
      
    }
    else if(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){
      junction="TB";
    }
    else{
      junction="TP";
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
    turnRight(sensorValues);
    Serial.println("Hiiii");

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnLeft(sensorValues);
    Serial.println("Hellooo");
  }
  else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}
*/
void turnRight(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < targetCounts && encoderCountB < targetCounts ) {
    
    motor1.setSpeed(120);
    motor1.forward();
    motor2.setSpeed(120);
    motor2.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  Serial.print("Encoder A Count: ");
  Serial.println(encoderCountA);
  Serial.print("Encoder B Count: ");
  Serial.println(encoderCountB);
}



void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountB < targetCounts && encoderCountA < targetCounts) {
    Serial.println(encoderCountA,encoderCountB);
 
    motor2.setSpeed(120);
    motor2.forward();
    motor1.setSpeed(120);
    motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  Serial.print("Encoder A Count: ");
  Serial.println(encoderCountA);
  Serial.print("Encoder B Count: ");
  Serial.println(encoderCountB);
}


void encoderISR_A() {
  encoderCountA++;
}

// Interrupt Service Routine (ISR) for Motor B encoder
void encoderISR_B() {
  encoderCountB++;
}