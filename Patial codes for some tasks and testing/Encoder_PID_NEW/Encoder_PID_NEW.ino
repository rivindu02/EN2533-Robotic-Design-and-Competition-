#include <L298N.h>

// PID parameters


// Motor control pins
#define PWMA 2
#define PWMB 7
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6
// Encoder pins

#define ENCODER_A1 18       // Encoder A channel (Interrupt pin)===> left

#define ENCODER_B1 20     // Encoder B channel (Interrupt pin)=====> right

L298N motor1(PWMA, AIN1, AIN2);     // left motor ,motor A, motor 1
L298N motor2(PWMB, BIN1, BIN2);

// Encoder counts
volatile long encoderCountA1 = 0;  // Encoder count for Motor A
volatile long encoderCountB1 = 0;  // Encoder count for Motor B
const long targetCounts = 190;   // Desired encoder counts for forward movement


// PID parameters for encoders
float Kpenco = 5;  // Proportional gain
float Kienco = 0;  // Integral gain
float Kdenco = 5;  // Derivative gain

// PID variables
float errorenco = 0;
float previousErrorenco = 0;
float integralenco = 0;
float derivativeenco = 0;

int baseSpeed=95;

void setup() {
  // Setup motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Setup encoder pins
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), countRight, RISING);

  Serial.begin(9600);
}

void loop() {
  moveStraightPID();
}

void moveStraightPID() {
  // Calculate the error (difference between encoder counts)

  errorenco = encoderCountA1 - encoderCountB1;
  Serial.println(errorenco);

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
  Serial.println(motor1Speed);
  Serial.println(motor2Speed);

  // Set motor speeds
  motor1.setSpeed(motor1Speed);
  motor1.forward();
  motor2.setSpeed(motor2Speed);
  motor2.forward();
  // Save the current error for the next iteration
  previousErrorenco = errorenco;
}


// Interrupt service routines for encoders
void countLeft() {
  encoderCountA1++;
}

void countRight() {
  encoderCountB1++;
}
