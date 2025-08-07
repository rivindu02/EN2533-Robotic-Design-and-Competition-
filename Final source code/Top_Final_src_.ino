#include <L298N.h>
#include <Servo.h>
#include <string.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h" 
#include <math.h>


Servo servo_grabbing; // Create a servo object
Servo servo_lifting;
int box_height;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// A====> left
// B====> Right


// A====> left
// B====> Right


#define PWMA 4
#define PWMB 9
#define AIN1 5
#define AIN2 6
#define BIN1 7
#define BIN2 8

#define trig1 26
#define echo1 27
#define trig2 22
#define echo2 23


// for IR_array
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
const int analogSensorPins_w[analogSensorCount] = {A15, A14, A13, A12, A11, A10, A9, A8};

int IR_weight[SensorCount] = {-50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];
int sensorValues_w[SensorCount];




// PID control parameters
float Kp = 5 ; // Proportional term
float Ki = 0; // Integral term
float Kd = 5;// Derivative term
float sum;
int increment_count = 1;

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
const int count_moveForward = 85;
const int count_moveForward_w = 125;


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

//int modulus=0;    // for experiment ONLY

// const int size_1b=3;
// const String Turn_1b[size_1b]={"RT","TP","TT"};
// const String Actions_1b[size_1b]={"180","L","B"};
// const String LED_1b[size_1b]={"N","N","S"};


// const int size_1r=11;
// const String Turn_1r[size_1r]={"RT","TP","LL","LT","LT","LT","LT","LT","TT","TP","TT"};
// const String Actions_1r[size_1r]={"180","F","L","L","F","F","B1","L","R","R","B"};
// const String LED_1r[size_1r]={"N","N","N","N","S","N","O","N","N","N","S"};

// const int size_2b=9;
// const String Turn_2b[size_2b]={"RT","TP","TT","LT","LT","LT","TT","TP","TT"};
// const String Actions_2b[size_2b]={"F","R","R","F","B1","R","L","L","B"};
// const String LED_2b[size_2b]={"N","N","N","S","O","N","N","N","S"};

// const int size_2r=9;
// const String Turn_2r[size_2r]={"RT","TP","TT","LT","LT","LT","TT","TP","TT"};
// const String Actions_2r[size_2r]={"180","L","L","F","B1","L","R","R","B"};
// const String LED_2r[size_2r]={"N","N","N","S","O","N","N","N","S"};

// Data Base

const int size_1b=3;
const String Turn_1b[size_1b]={"RT","TP","TT"};
const String Actions_1b[size_1b]={"180","L","B"};
const String LED_1b[size_1b]={"N","N","S"};


const int size_1r=11;
const String Turn_1r[size_1r]={"RT","TP","LL","LT","LT","LT","LT","LT","TT","TP","TT"};
const String Actions_1r[size_1r]={"180","F","L","L","F","F","B1","L","R","R","B"};
const String LED_1r[size_1r]={"N","N","N","N","S","N","O","N","N","N","S"};

const int size_2b=9;
const String Turn_2b[size_2b]={"RT","TP","TT","RT","RT","RT","TT","TP","TT"};
const String Actions_2b[size_2b]={"F","R","R","F","B1","R","L","L","B"};
const String LED_2b[size_2b]={"N","N","N","S","O","N","N","N","S"};

const int size_2r=9;
const String Turn_2r[size_2r]={"RT","TP","TT","LT","LT","LT","TT","TP","TT"};
const String Actions_2r[size_2r]={"180","L","L","F","B1","L","R","R","B"};
const String LED_2r[size_2r]={"N","N","N","S","O","N","N","N","S"};


const int size_3b = 11;
const String Turn_3b[size_3b]={"RT","TP","RR","RR","RT","RT","RT","RT","TT","TP","TT"};
const String Actions_3b[size_3b]={"F","F","R","R","F","F","B1","R","L","L","B"};
const String LED_3b[size_3b]={"N","N","N","N","S","N","O","N","N","N","S"}; 

const int size_3r=3;
const String Turn_3r[size_3r]={"RT","TP","TT"};
const String Actions_3r[size_3r]={"F","R","B"};
const String LED_3r[size_3r]={"N","N","S"};

const int size_4b=16;
const String Turn_4b[size_4b]={"RT","TT","LT","LL","LT","LT","TP","RT","TP","TT","RT","RT","RT","TT","TP","TT"};
const String Actions_4b[size_4b]={"R","L","F","B2","B1","L","R","F","R","R","F","B1","R","L","L","B"};
const String LED_4b[size_4b]={"N","N","N","S","O","N","N","N","N","N","S","O","N","N","N","S"};

const int size_4r=9;
const String Turn_4r[size_4r]={"RT","TT","LT","LL","LT","LT","TT","TP","TT"};
const String Actions_4r[size_4r]={"R","L","F","B1","B1","L","R","R","B"};
const String LED_4r[size_4r]={"N","N","N","S","O","N","N","N","S"};






// Actions
// B1----> backward 1 line----> Array should pass the junction and stop delay 500
// B2----> backward 2 line
// B4----> backward 4 line
// S-----> stop for look for the wall

int Array_size;

// for Line Navigation
int No_lines=0;
int modulus;
bool Code[20]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int pulses_array[20]={0};
int first_black_line=0;

// Box_arrangement
String Colour;



void setup() {

  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins_w[i], INPUT);
  }


  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(53, OUTPUT);        //  led indicate in virtual box 

  // Encoder pins setup
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);


  // give pinmode to defined leds

  //give initial values to servo motors

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(49,OUTPUT);
  

  pinMode(10,INPUT);    // IR pin mounted on arm
  pinMode(47,OUTPUT);   // potral indication

  digitalWrite(47,LOW);


  //digitalWrite(53,OUTPUT);    //blue
  //digitalWrite(51,OUTPUT);

  //backward_B3();
  Serial.begin(9600);





  // new servo pins
  pinMode(10,INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13 , LOW);
  servo_lifting.attach(12);
  servo_grabbing.attach(11); // Attach the servo to pin 9
  servo_lifting.attach(12);

}

void loop() {
  //move();


 // Call the PID function to keep the robot moving straight
 /*moveStraightPID();

  // Add a small delay to control the PID loop frequency
 delay(10);*/

  // dashLine();
  // if (increment_count==1){
  //   portal();

  // }

  //Wall_detect();
    //   if (Wall_detect()){
    //   digitalWrite(49,HIGH);
    // }else{
    //   digitalWrite(49,LOW);
    // }

  switch (increment_count){
    case 1:
      LineNavigation();
      break;
    case 2:
      VirtualBox(modulus);
      break;
    case 3:
      colour_line_follow();
      break;
    case 4:
      dashLine();
      break;
    case 5:
      portal();
      break;
  }


  //   readSensors_w(sensorValues_w);

  //   moveForward(sensorValues_w,count_moveForward_w);
  //   line_follow(sensorValues_w);

  //   if (junction=="LT"){
  //     turnLeft();
  //   }
  //   junction="a";
  
  // if (digitalRead(10) == LOW){
  //   motor1.stop();
  //   motor2.stop();
  //   boxlift();
  //   half_rotation();
  //   while(true){
  //     String Action[3]={"L","F","R"};
  //     String Turns[3]={"TT","RT","RT"};
  //     String LED[3]={"N","N","N"};
  //     Array_size=3;
  //     find_path_full_array_(Turns,Action,LED);
  //     delay(1000);
  //     boxdrop();

  //   }
  // }
  
    
 
}

void find_path_full_array_(String Turn[], String Actions[],String LED[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors_w(sensorValues_w);
      line_follow(sensorValues_w);
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





void boxlift(){
  servo_grabbing.write(28); // Set servo to 90 degrees (neutral position)
  servo_lifting.write(10);
  delay(3000);
  for(int i = 28; i<=(90); i++){
    servo_grabbing.write(i);
    delay(25);
  } 
  delay(2000);

  for(int i = 10; i<=(50); i++){
    servo_lifting.write(i);
    delay(100);
  } 
  delay(3000);

}
void boxdrop(){
  servo_grabbing.write(90); // Set servo to 90 degrees (neutral position)
  servo_lifting.write(50);
  delay(3000);
  for(int j = (50); j>=10; j--){
    servo_lifting.write(j);
    delay(200);

  }
  delay(2000);
  for(int j = (90); j>=28; j--){
    servo_grabbing.write(j);
    delay(25);

  }
  delay(3000);

}

void boxheight(){
  servo_lifting.write(10);
  for(int i = 10; i<=(50); i++){
    servo_lifting.write(i);
    delay(100);
  }
  delay(1000);
  
  if (digitalRead(10) == LOW){
    box_height = 15;
    Serial.println("Hiiii");
    Serial.println(box_height);
    while(1);
  }
  else{
    for(int i = 50; i>=(25); i--){
      servo_lifting.write(i);
      delay(100);
    }
    delay(1000);
    if (digitalRead(10)== LOW){
      box_height = 10;
      Serial.println("Heeee");
      Serial.println(box_height);
      while(1);
    }
    else{
      box_height = 5;
      Serial.println("Hoooo");
      Serial.println(box_height);
      while(1);
    }
  }

}











void portal(){

  while (digitalRead(10) == HIGH){
    motor1.stop();
    motor2.stop();
    digitalWrite(47 , HIGH);    
  }
  if (digitalRead(10) == LOW){
    delay(5000);
    moveStraightPID();
    digitalWrite(47,LOW);
  }

}


void dashLine(){
  readSensors(sensorValues);
  float pidValue = calculatePID(sensorValues);
  PID_Linefollow(pidValue);
  if (sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7] < 4) {
    while (encoderCountA < 100 && encoderCountB < 100 ) {
  
      moveStraightPID();
      

      // motor1.setSpeed(115);
      // motor1.forward();
      // motor2.setSpeed(115);
      // motor2.forward(); 
    }
    readSensors(sensorValues);
  }
  line_follow(sensorValues);
  
  if (junction=="TB"){
    encoderCountA=0;
    encoderCountB=0;
    while (encoderCountA < 15 && encoderCountB < 15 ) {
    
    moveStraightPID();
  }
    motor1.stop();
    motor2.stop();
    increment_count++;
    delay(3000);
  }

  junction="a";
  
}
// For Line Navigation


void LineNavigation(){
  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA2);
  //Serial.print("\t");
  //Serial.println(encoderCountB2);

  //delay(100);  // Update every 100 ms for get the count


   //motor1.setSpeed(90);
   //motor1.forward();
   //motor2.setSpeed(110);
   //motor2.forward();

  moveStraightPID();

  // for white-->1
  // for black-->0

  // for minimize the Error get the sum and find whether it is greater than 4 for less than 4

  // have to get count roughly for 3cm and 6cm



  /////////////////////////////////////// Should change these
  //int encoder_pulse_3cm=41;
  //int encoder_pulse_6cm=75;
  //int threshold =59;//(encoder_pulse_3cm + encoder_pulse_6cm)/2;

  //readSensors(sensorValues);
  bool current_line= Identify_Strip(sensorValues); //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
  
  
  if (current_line==0){//black
    first_black_line++;
  }
  if(current_line==1 && first_black_line>0){  //white and first black line encounters 
    encoderCountA=0;
    encoderCountB=0;
    
    while(current_line==1){
      moveStraightPID();
      current_line= Identify_Strip(sensorValues);
    }
    if (encoderCountB>10){
      No_lines++;
      if (encoderCountB>59){
        Code[No_lines-1]=1;
      }if(encoderCountB<59 && 20<encoderCountB){
        Code[No_lines-1]=0;
      }

    }
    
    //pulses_array[No_lines]=encoderCountB;
    
    // Serial.print("Pulses: ");
    // Serial.print(encoderCountB);
    // Serial.print("\t");
    // Serial.print("lines: ");
    // Serial.print(No_lines);
    // Serial.println();

  }
    
  // for  (int i=0; i<20; i++){
  //   if (pulses_array[i]>59){
  //     Code[True_Lines]=1;
  //     True_Lines++;
      
  //   }
  //   if (pulses_array[i]<59 && 29<pulses_array[i]){
  //     Code[True_Lines]=0;
  //     True_Lines++;
  //   }
  // }
    //delay(200);    
  

  if (No_lines>3){
    if (Code[No_lines-1]==0 && Code[No_lines-2]==0 && Code[No_lines-3]==0){
      
      bool Final_Code[No_lines-3];
      // number of lines in Final Code=(No_lines-3)

      for (int i=0; i<No_lines-4; i++){     // 0,1,2,3,.....,(No_lines-4) =====> // Final_Code lenght is (No_lines-3) 
        Final_Code[i]=Code[i+1];
      }
      
      motor1.stop();
      motor2.stop();
      delay(500);
      //computeModulus(int* code);
      increment_count++;
      int value_of_barcode=0;
      for (int j=0; j<(No_lines-4) ; j++){
        //Serial.print(Final_Code[j]);
        //Serial.print("\t");
        
          //Serial.print(value_of_barcode);
        value_of_barcode=value_of_barcode*2+Final_Code[j];
        
      }

      Serial.println(value_of_barcode);
      modulus=value_of_barcode % 5;
      Serial.println(modulus);



      //Serial.println(No_lines);
    }
  }

}











void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 80 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
  }
}

void readSensors_w(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins_w[i]) > 100 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
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
    againcheck(sensorValues,count_moveForward);
    
      
  }
  else if (isImmediateTurnR(sensorValues)) {
    //junction = "R";
    motor1.stop();
    motor2.stop();
    delay(5);   //500
    //digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues,count_moveForward);
    //moveForward(sensorValues);
  }
  else if (isImmediateTurnT(sensorValues)) {
    //junction = "T";
    motor1.stop();
    motor2.stop();
    delay(5);     // 500
    //digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues,count_moveForward);
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

void againcheck(int *sensorValues,int count){
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
    moveForward(sensorValues,count);

  }
  else if (isImmediateTurnR(sensorValues)) {
    junction = "R";
    motor1.stop();
    motor2.stop();
    delay(5);   // 500
    //digitalWrite(26, HIGH);
    //delay(2000);
    moveForward(sensorValues,count_moveForward);
  }
  else if (isImmediateTurnT(sensorValues)) {
    junction = "T";
    motor1.stop();
    motor2.stop();
    delay(5);   // 500
    //digitalWrite(26, HIGH);
    //delay(20)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              00);
    moveForward(sensorValues,count_moveForward);
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




void moveForward(int *sensorValues, int count) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  //Serial.print("First");
  //Serial.println(junction);
  readSensors(sensorValues);
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < count && encoderCountB < count ) {
    
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

void colour_line_follow(){      // testing

  readSensors(sensorValues);
  line_follow(sensorValues);

  if (junction=="LL"){
    //Serial.println(junction);
    turnLeft();
  }
  else if(junction=="LT"){
    //Serial.println(junction);
    line_follow(sensorValues);
    //turnLeft();
  }
  else if(junction=="RR"){
    //Serial.println(junction);
    turnRight();
  }
  else if(junction=="RT"){
    //Serial.println(junction);
    line_follow(sensorValues);
    //turnRight();
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
    delay(3000);
    //line_follow(sensorValues);
    increment_count++;
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

void VirtualBox(int modulus){
  if (modulus==0){
    String Path[8]={"RR","RT","RR","RT","LT","LT","TT","TP"};
    String Actions[8]={"R","F","B1","R","F","L","L","S"};
    String LED[8]={"N","S","O","N","N","N","N","N"};
    Array_size=8;
    bool wall_there=Wall_detect();           //red case
    find_path_full_array(Path,Actions,LED);
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
    bool wall_there=Wall_detect(); // blue case
    find_path(Path,Actions);
    // check blue wall 
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
          Array_size=size_3r;
          find_path_full_array(Turn_3r,Actions_3r,LED_3r);
          break;
        case 4:
          Array_size=size_4r;
          find_path_full_array(Turn_4r,Actions_4r,LED_4r);
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
          Array_size=size_3b;
          find_path_full_array(Turn_3b,Actions_3b,LED_3b);
          break;
        case 4:
          Array_size=size_4b;
          find_path_full_array(Turn_4b,Actions_4b,LED_4b);
          break;
      }
    }
  }
  

}









// D53-blue , D51
String colordetect(){
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  
  if (b > r){
    //digitalWrite(53,HIGH);
    //digitalWrite(51,LOW);
    return "Blue" ;
  } 
  if (b < r){
    //digitalWrite(51,HIGH);
    //digitalWrite(53,LOW);
    return "Red";
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
    //digitalWrite(53,HIGH);
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
    increment_count++;
    Colour=colordetect();
    digitalWrite(53,LOW);
    half_rotation();
    moveForward(sensorValues,count_moveForward);
    
  }
    if (Action=="FF"){
    while(Blue_Red_NOTdetect()){
      //colordetect1();
      moveStraightPID();
    }
    motor1.stop();
    motor2.stop();
    delay(3000);
    digitalWrite(53,LOW);
    increment_count++;
    Colour=colordetect();

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

//////////////////


bool Wall_detect(){
  digitalWrite(trig1, LOW);

  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  
  long t1 = pulseIn(echo1, HIGH);
  
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);

  
  long t2 = pulseIn(echo2, HIGH);

  long cm1 = t1/29/2;
  long cm2 = t2/29/2;

  //Serial.print(cm1);
  //Serial.print("\t");
  //Serial.print(cm2);
  //Serial.println();

  // if((2<cm1) && (cm1<70) || (2<cm2) && (cm2<70)){
  //     digitalWrite(49,HIGH);
  //     delay(1000);
  //     digitalWrite(49,LOW);
  // }

  //delay(100);
  return ((2<cm1) && (cm1<90) || (2<cm2) && (cm2<90) );
}









void LED_Switching_Juc(String LEDstate){
  // S---> Switch On
  // O---> Switch Off
  // N---> Nothing
  if (LEDstate=="S"){
    // Turn on LED
    digitalWrite(53,HIGH);
  }
  if (LEDstate=="O"){
    // Turn Off LED
    digitalWrite(53,LOW);
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

void forward_1() {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < 450 && encoderCountB < 450 ) {
    
    moveStraightPID();
    // motor1.setSpeed(115);
    // motor1.forward();
    // motor2.setSpeed(115);
    // motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000); // 4000
}




void Identify_action_w(String Action){

  if (Action=="180"){half_rotation();}

  if (Action=="L"){turnLeft();}

  if (Action=="R"){turnRight();}

  if (Action=="FF"){
    forward_1();
    
  }
  if (Action=="S"){
    motor1.stop();
    motor2.stop();
    delay(3000);
    }

  if (Action=="F"){
    while(Box_Detection()){
      line_follow(sensorValues);
    }
  }

  if (Action=="B1"){backward_B1();}

  if (Action=="B2"){backward_B2();}

  if (Action=="B3"){backward_B3();}


}

// void find_path_w(String Actions[]){
//     for (int i=0; i<Array_size; i++){
//       if (Actions[i]=="L"|| Actions[i]=="R"){
//         while(true){
//           readSensors_w(sensorValues_w);
//           line_follow(sensorValues_w);
//       //String passed_turn=Turn[i];
//           if ("TB"==junction){
//             Identify_action_w(Actions[i]);
//             junction="a";
//             break;
//           }
//         }
  
//       }else{
//         Identify_action_w(String Action[i]);                // Descending 



//       }



//   }


// }








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



String Arranging_order(String Colour){
  if (Colour=="Blue"){return "Ascending";}      // Ascending order
  if (Colour=="Red"){return "Descending";}
}

//  Box_Detection function should feed Ultra Sound input



bool Box_Detection(){   // if detect return false

  return 0;

}


void Box_lift(){}

void Box_drop(){}


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
void test(){
  while(digitalRead(10)==LOW){
      readSensors_w(sensorValues_w);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (junction=="TB"){
        Identify_action_w("L");  
      }
      junction="a";

  }

}




// void find_path_Box_Arrangement(String Actions[]){


//   for (int i=0; i<Array_size; i++){
//     while(true){
//       readSensors_w(sensorValues_w);
//       line_follow(sensorValues);
//       //String passed_turn=Turn[i];
//       if (Turn[i]==junction){
//         Identify_action(Actions[i]);
//         break;
//         //Mech_Arm(Drop[i]);
//       }
//       if (junction=="TB"){        // white Box*********** DON't INVERT
//         break;

//       }
//       junction="a";
//     }
//   }

// }


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