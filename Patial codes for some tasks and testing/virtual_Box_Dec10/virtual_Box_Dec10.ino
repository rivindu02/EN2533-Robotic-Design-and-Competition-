#include <L298N.h>
#include <Servo.h>
#include <string.h>

// A====> left
// B====> Right


#define PWMA 2
#define PWMB 7
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6
// 23,25..29

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
int baseSpeed = 95;

String junction = "a";


// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();

L298N motor1(PWMA, AIN1, AIN2);     // left motor ,motor A, motor 1
L298N motor2(PWMB, BIN1, BIN2);

#define ENCODER_A1 18       // Encoder A channel (Interrupt pin)===> left----> encoder PID
#define ENCODER_B1 20     // Encoder B channel (Interrupt pin)=====> right---> encoder PID

// Encoder counts
volatile long encoderCountA1 = 0;  // Encoder count for Motor A
volatile long encoderCountB1 = 0;  // Encoder count for Motor B
const long targetCounts = 190;            // Desired encoder counts for 90 degree turns
const long targetCounts_rotation = 410;   // Desired encoder counts for 180 degree turns




// PID parameters for encoders
float Kpenco = 5;  // Proportional gain
float Kienco = 0;  // Integral gain
float Kdenco = 5;  // Derivative gain

// PID variables for encoders
float errorenco = 0;
float previousErrorenco = 0;
float integralenco = 0;
float derivativeenco = 0;

// for Line Navigation
int No_lines=0;




// For Virtual box ----> After getting modulus from LineNavigation

int modulus=1;    // for experiment ONLY

const int size_1b=2;
const String Turn_1b[size_1b]={"TP","TT"};
const String Actions_1b[size_1b]={"R","B"};
const String LED_1b[size_1b]={"N","S"};


const int size_1r=10;
const String Turn_1r[size_1r]={"TP","LL","LT","LT","LT","LT","RT","TT","TP","TT"};
const String Actions_1r[size_1r]={"180","L","L","F","F","180","R","R","R","B"};
const String LED_1r[size_1r]={"N","N","N","S","N","O","N","N","N","S"};

int const Array_size;


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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), countRight, RISING);


  half_rotation();
  //motor1.stop();
  //motor2.stop();

  // give pinmode to defined leds

  //give initial values to servo motors

  Serial.begin(9600);


  pinMode(23,OUTPUT);
  pinMode(25,OUTPUT);
  pinMode(27,OUTPUT);
  pinMode(29,OUTPUT);

}

void loop() {
 //move();
 //VirtualBox(modulus);

}
void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
  }
}
void line_follow(int *sensorValues) {
  
  if (isImmediateTurnL(sensorValues)) {
    //junction = "L";
    motor1.stop();
    motor2.stop();
    delay(500);
    
    digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    
  }
  else if (isImmediateTurnR(sensorValues)) {
    //junction = "R";
    motor1.stop();
    motor2.stop();
    delay(500);
    digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    //moveForward(sensorValues);
  }
  else if (isImmediateTurnT(sensorValues)) {
    //junction = "T";
    motor1.stop();
    motor2.stop();
    delay(500);
    digitalWrite(26, HIGH);
    //delay(2000);
    againcheck(sensorValues);
    //moveForward(sensorValues);
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
  lsp = baseSpeed - pidValue;
  rsp = baseSpeed + pidValue;

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
  encoderCountA1 = 0;
  encoderCountB1 = 0;

  while (encoderCountA1 < 15 && encoderCountB1 < 15 ) {
    
    motor1.setSpeed(95);
    motor1.forward();
    motor2.setSpeed(115);
    motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
  readSensors(sensorValues);
  if (isImmediateTurnL(sensorValues)) {
    junction = "L";
    motor1.stop();
    motor2.stop();
    delay(500);
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
void moveForward(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  Serial.println(junction);
  readSensors(sensorValues);
  encoderCountA1 = 0;
  encoderCountB1 = 0;

  while (encoderCountA1 < 85 && encoderCountB1 < 85 ) {
    
    motor1.setSpeed(95);
    motor1.forward();
    motor2.setSpeed(115);
    motor2.forward();
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
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
    encoderCountA1 = 0;
    encoderCountB1 = 0;

    while (encoderCountA1 < 75) {
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
  encoderCountA1 = 0;
  encoderCountB1 = 0;

  while (encoderCountA1 < targetCounts && encoderCountB1 < targetCounts ) {
    
    motor1.setSpeed(120);
    motor1.forward();
    motor2.setSpeed(120);
    motor2.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA1);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB1);
}

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA1 = 0;
  encoderCountB1 = 0;

  while (encoderCountB1 < targetCounts && encoderCountA1 < targetCounts) {
    Serial.println(encoderCountA1,encoderCountB1);
 
    motor2.setSpeed(120);
    motor2.forward();
    motor1.setSpeed(120);
    motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA1);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB1);
}


void countLeft() {
  encoderCountA1++;
}

// Interrupt Service Routine (ISR) for Motor B encoder
void countRight() {
  encoderCountB1++;
}

void move(){

  readSensors(sensorValues);
  line_follow(sensorValues);

  if (junction=="LL"){
    //Serial.println(junction);
    turnLeft(sensorValues);
  }
  else if(junction=="LT"){
    //Serial.println(junction);
    //line_follow(sensorValues);
    turnLeft(sensorValues);
  }
  else if(junction=="RR"){
    //Serial.println(junction);
    turnRight(sensorValues);
  }
  else if(junction=="TP"){
    //Serial.println(junction);
    line_follow(sensorValues);
    //turnRight(sensorValues);
  }
  else if(junction=="TT"){
    //Serial.println(junction);
    turnRight(sensorValues);
    //turnLeft(sensorValues);
  }
  else if(junction=="TB"){
    //Serial.println(junction);
    motor1.stop();
    motor2.stop();
    while (1);
  }
  junction="a";
}

void VirtualBox(int modulus){
  
  while(true){
    readSensors(sensorValues);
    line_follow(sensorValues);
    
    if(junction=="RR"){
      turnRight(sensorValues);
    }
    if (junction=="RT"){
      junction="a";         // new modification
      if (modulus==0){

      }else{                        // for modulus=1,2,3,4
        while(true){
          readSensors(sensorValues);
          line_follow(sensorValues);
          if (junction=="RR"){
            turnRight(sensorValues);
          }

          if (junction=="TP"){
            const int Array_size =size_1b;
            motor1.stop();
            motor2.stop();
            find_path(Turn_1b,Actions_1b,LED_1b);


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

  if (Action=="L"){turnLeft(sensorValues);}

  if (Action=="R"){turnRight(sensorValues);}

  if (Action=="B"){half_rotation();}

  if (Action=="F"){line_follow(sensorValues);}

}

void half_rotation() {
  // Perform a sharp left turn by 
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA1 = 0;
  encoderCountB1 = 0;

  while ((encoderCountB1 < targetCounts_rotation) && (encoderCountA1 < targetCounts_rotation)) {
    Serial.println(encoderCountA1,encoderCountB1);
 
    motor2.setSpeed(120);
    motor2.forward();
    motor1.setSpeed(120);
    motor1.backward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  //Serial.print("Encoder A Count: ");
  //Serial.println(encoderCountA1);
  //Serial.print("Encoder B Count: ");
  //Serial.println(encoderCountB1);
}



// Robo should go backward until the box should increment the increment count and go to next task
void encoder_backward(){

    // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA1 - encoderCountB1;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kpenco * errorenco) + (Kienco * integralenco) + (Kdenco * derivativeenco);

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


// Not for this USE this to build encoder_backward()
void moveStraightPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA1 - encoderCountB1;
  //Serial.println(errorenco);

  // Calculate the integral and derivative terms
  integralenco += errorenco;
  derivativeenco = errorenco - previousErrorenco;

  // Compute the PID correction
  float correction = (Kpenco * errorenco) + (Kienco * integralenco) + (Kdenco * derivativeenco);

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


void LED_Switching_Juc(String LEDstate){
  // S---> Switch On
  // O---> Switch Off
  // N---> Nothing
  if (LEDstate=="S"){
    // Turn on LED
    digitalWrite(23,HIGH);
  }
  if (LEDstate=="O"){
    // Turn Off LED
    digitalWrite(23,LOW);
  }
}



void find_path(String Turn[], String Actions[],String LED[]){
  for (int i=0; i<Array_size; i++){
    while(true){
      readSensors(sensorValues);
      line_follow(sensorValues);
      //String passed_turn=Turn[i];
      if (Turn[i]==junction){
        Identify_action(Actions[i]);
        LED_Switching_Juc(LED[i]);
        break;
      }
      junction="a";
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
        turnLeft(sensorValues);
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

          find_path_Box_Arrangement(Turn_3A,Action_3A);
          Box_drop();

          while(true){
            readSensors(sensorValues);
            line_follow(sensorValues);

            if  (junction=="TP"){
              turnLeft(sensorValues);

              while(true){
                readSensors(sensorValues);
                line_follow(sensorValues);

                if  (junction=="TP"){
                  turnRight(sensorValues);

                }

                if (Box_Detection()){
                  motor1.stop();
                  motor2.stop();
                  if  (Hight_Measure()==1){
                    delay(7000);
                    Box_lift();
                    half_rotation();

                    const String Turn_1B[]={"TP","TP"};
                    const String Turn_1B[]={"R","L"};

                    find_path_Box_Arrangement(Turn_1B,Action_1B);
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
