#include <L298N.h>

// PID parameters
float Kp = 9.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain

// Variables
volatile int encoderCountA = 0;    // Encoder counts for left motor
volatile int encoderCountB = 0;   // Encoder counts for right motor
int targetSpeed = 100;           // Target speed (encoder counts per time step)
float error, lastError = 0;
float integral = 0;
float derivative = 0;
float correction;

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


// Timer variables
unsigned long lastTime = 0;
unsigned long interval = 100; // 100 ms control loop interval

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
  go_straight();
}

void go_straight(){
  unsigned long currentTime = millis();

  // Run the PID control loop at specified intervals
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Calculate error (difference in encoder counts)
    error = encoderCountA - encoderCountB;

    // PID computations
    integral += error;
    derivative = error - lastError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    // Adjust motor speeds based on correction
    int leftMotorSpeed = constrain(targetSpeed - correction, 0, 255);
    int rightMotorSpeed = constrain(targetSpeed + correction, 0, 255);

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    // Reset encoder counts for next iteration
    encoderCountA = 0;
    encoderCountB = 0;

    // Debugging info
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" Correction: ");
    Serial.println(correction);
  }
}


// Motor speed control function
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  motor1.setSpeed(leftSpeed);
  motor1.forward();

  motor2.setSpeed(rightSpeed);
  motor2.forward();

}

// Interrupt service routines for encoders
void countLeft() {
  encoderCountA++;
}

void countRight() {
  encoderCountB++;
}
