#include <L298N.h>
#include <Servo.h>

//////////////
// Have to redefine those below pins as we plugged
// change variable name according to name mentioned in board
#define PWMA 2
#define PWMB 7
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6
//motor1=right
////////


// for IR_array
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
int IR_weight[SensorCount] = {-50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];




// PID control parameters
float Kp = 12 ; // Proportional term
float Ki = 0.02; // Integral term
float Kd = 5; // Derivative term
float sum;
int increment_count = 0;

long ldistance;
long rdistance;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int lfspeed = 85;


//ultrasonic sensor read
//////ultrasonic pins
#define lftrig 10
#define lfecho 11
#define rftrig 8
#define rfecho 9


///// Encoders (only use 2, 3, 18, 19, 20, and 21)
#define encoderA1 18     // Encoder 1 signal of motor A
#define encoderA2 19     // Encoder 2 signal of motor A
#define encoderB1 20     // Encoder 1 signal of motor A
#define encoderB2 21     // Encoder 2 signal of motor A

volatile int countA = 0;  // Encoder count
volatile int countB = 0;

volatile bool lastA1State = LOW;
volatile bool lastA2State = LOW;

volatile bool lastB1State = LOW;
volatile bool lastB2State = LOW;



//define leds as mentioned in tasks /////////

// lcd display implementation


// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();

// Assign pin numbers to each servo
const int servo1 = 4;
const int servo2 = 5;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

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

  pinMode(lftrig, OUTPUT);
  pinMode(lfecho, INPUT);
  pinMode(rftrig, OUTPUT);
  pinMode(rfecho, INPUT);


  motor1.stop();
  motor2.stop();

  // give pinmode to defined leds

  //give initial values to servo motors

  Serial.begin(9600);

}

void loop() {
  readSensors(sensorValues);
  line_follow(sensorValues);

}

void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
  }
}



void line_follow(int *sensorValues) {
  /*if (isImmediateTurn(sensorValues)) {
    motor1.stop();
    motor2.stop();
    //digitalWrite(26, HIGH);
    //delay(2000);
    turn(sensorValues);
  }
  else if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    digitalWrite(30, HIGH);
    noLine(sensorValues);
    //digitalWrite(8, LOW);
  }
  else {

  }     */
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);

    // You may adjust the delay for performance tuning
    //digitalWrite(26, LOW);
    //digitalWrite(28, HIGH);
    //digitalWrite(30, LOW);
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
    rsp = -255;           //**************** initial value = 255
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


void LineNavigation(){



}













bool isImmediateTurn(int *sensorValues) {
  bool turnLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1;
  bool turnRight = sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return (turnLeft || turnRight);
}
void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    turnLeft(sensorValues);

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnRight(sensorValues);
  }
  else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}




//case

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(100);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(100);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
    //digitalWrite(26, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(100);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(100);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnLeft_T(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(95);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(120);   // Set left motor speed
    motor2.backward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void turnRight_T(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!enoughturn && !whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn || whitedetect) {
      break;
    }
    motor1.setSpeed(120);   // Set right motor speed
    motor1.backward();       // Move right motor forward
    motor2.setSpeed(120);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    digitalWrite(28, HIGH);
  }
  motor1.stop();
  motor2.stop();
  //delay(1000);
}

void noLine(int *sensorValues) {
  float firstNonZeroValue;
  for (int i = 0; i < 50; i++)
  {
    if (errorArray[i] != 0) {
      firstNonZeroValue = errorArray[i];
      break; //
    }
  }

  if (firstNonZeroValue < 0) {
    turnRight(sensorValues);

  }
  else if (firstNonZeroValue > 0) {
    turnLeft(sensorValues);

  }
}


void white(int *sensorValues) {
  //  motor1.setSpeed(55);   // Set right motor speed
  //  motor1.forward();       // Move right motor forward
  //  motor2.setSpeed(55);   // Set left motor speed
  //  motor2.forward();
  //  delay(400);
  //  motor1.stop();
  //  motor2.stop();
  //  delay(200);
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    motor1.setSpeed(80);   // Set right motor speed
    motor1.forward();       // Move right motor forward
    motor2.setSpeed(80);   // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  increment_count += 1;
}

long lUltrasonic_read() {
  digitalWrite(lftrig, LOW);
  delayMicroseconds(2);
  digitalWrite(lftrig, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (lfecho, HIGH);
  ldistance = time / 29 / 2;
  return ldistance;
}
long rUltrasonic_read() {
  digitalWrite(rftrig, LOW);
  delayMicroseconds(2);
  digitalWrite(rftrig, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (rfecho, HIGH);
  rdistance = time / 29 / 2;
  return rdistance;
}

