#include <L298N.h>
#include <Servo.h>

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
float Kp = 10 ; // Proportional term
float Ki = 0.02; // Integral term
float Kd = 8;// Derivative term
float sum;
int increment_count = 0;

long ldistance;
long rdistance;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 85;

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
const long targetCounts = 390;   // Desired encoder counts for forward movement

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
  readSensors(sensorValues);
  line_follow(sensorValues);

}

void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135)black ====> return 0 (no line)
  }
}

void line_follow(int *sensorValues) {
  
  if (isImmediateTurn(sensorValues)) {
    
    motor1.stop();
    motor2.stop();
    delay(500);
    /*motor1.setSpeed(85);
    motor1.forward();
    motor2.setSpeed(85);
    motor2.forward();
    delay(350);*/

    digitalWrite(26, HIGH);
    //delay(2000);
    turn(sensorValues);
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

bool isImmediateTurn(int *sensorValues) {
  bool turnLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0 ;
  bool turnRight = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return (turnLeft || turnRight);
}

void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  readSensors(sensorValues);
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1){
    encoderCountA = 0;
    encoderCountB = 0;

    while (encoderCountA < 75) {
      motor1.setSpeed(85);
      motor1.forward();
      motor2.setSpeed(85);
      motor2.forward();
    }
    motor1.stop();
    motor2.stop();

  }

  else if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    turnLeft(sensorValues);

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnRight(sensorValues);
  }
  else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountA < targetCounts ) {
    motor2.stop();
    motor1.setSpeed(120);
    motor1.forward();
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
  Serial.print("Encoder A Count: ");
  Serial.println(encoderCountA);
  Serial.print("Encoder B Count: ");
  Serial.println(encoderCountB);
}

void turnRight(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  encoderCountA = 0;
  encoderCountB = 0;

  while (encoderCountB < targetCounts) {
    motor1.stop();
    motor2.setSpeed(120);
    motor2.forward();
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