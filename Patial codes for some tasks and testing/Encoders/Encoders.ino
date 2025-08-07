#include <L298N.h>

// A====> left
// B====> Right

#define PWMA 2
#define PWMB 7
#define AIN1 3
#define AIN2 4
#define BIN1 5
#define BIN2 6


/////////////////// which is A and which is B
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
float Kd = 5;// Derivative term
float sum;
int increment_count = 0;


int lfspeed = 85;

// for Line Navigation

///// Encoders (only use 2, 3, 18, 19, 20, and 21)
#define encoderA1 18     // Encoder 1 signal of motor A
#define encoderA2 19     // Encoder 2 signal of motor A
#define encoderB1 20     // Encoder 1 signal of motor A
#define encoderB2 21     // Encoder 2 signal of motor A







volatile int countA = 0;  // Encoder count
volatile int countB = 0;


// volatile bool lastA1State = LOW;
// volatile bool lastA2State = LOW;

// volatile bool lastB1State = LOW;
// volatile bool lastB2State = LOW;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);


// for Line Navigation
int No_lines=0;

void setup() {
  Serial.begin(9600);

  // Motor Encoder Pins
  pinMode(encoderA1, INPUT_PULLUP);     // don't know whether its pullup or pulldown
  pinMode(encoderA2, INPUT_PULLUP);
  pinMode(encoderB1, INPUT_PULLUP);
  pinMode(encoderB2, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderA1), encoderAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), encoderBInterrupt, CHANGE);

}

void loop() {
  
  Serial.print("Pulse Count: ");
  Serial.print(countA);
  Serial.print("\t");
  Serial.println(countB);

  delay(100);  // Update every 100 ms for get the count
  
}


void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
  }
}


void LineNavigation(int *sensorValues){
  Serial.print("Pulse Count: ");
  Serial.print(countA);
  Serial.print("\t");
  Serial.println(countB);

  delay(100);  // Update every 100 ms for get the count

  motor_drive(85,85);

  // for white-->0
  // for black-->1

  // for minimize the Error get the sum and find whether it is greater than 4 for less than 4

  // have to get count roughly for 3cm and 6cm

  bool Code[30]={0};

  int encoder_pulse_3cm;
  int encoder_pulse_6cm;
  float threshold =(encoder_pulse_3cm + encoder_pulse_6cm)/2;

  bool current_line= sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
  
  delay(50);

  bool new_line=sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7] ;

  // prev condition current_line=!new_line
  while (current_line==1 && new_line==0){ //enter into white strip
    No_lines++;
    countA=0;
    countB=0;
    while (new_line==0){
      delay(50);
      new_line=sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
    }
    int encoder_pulse=(countA+countB)/2;
    
    if (encoder_pulse>threshold){
      Code[No_lines-1]=1;
    } else{
      Code[No_lines-1]=0;
    }
    //delay(200);    
  }

  if (No_lines>3){
    if ((No_lines-1)==0 && (No_lines-2)==0 && (No_lines-3)==0){
      
      bool Final_Code[No_lines];
      
      for (int i=0; i=No_lines-1; i++){
        Final_Code[i]=Code[i];
      }

      increment_count++;
    }
  }


}


// Interrupt Service Routine (ISR) for encoder
void encoderAInterrupt() {
  //Serial.println("Interrupt Triggered");  // Debugging
  int stateA1 = digitalRead(encoderA1);
  int stateA2 = digitalRead(encoderA2);

  if (stateA1 == stateA2) {
    countA++;
  } else {
    countA--;
  }
}
void encoderBInterrupt() {
  //Serial.println("Interrupt Triggered");  // Debugging
  int stateB1 = digitalRead(encoderB1);
  int stateB2 = digitalRead(encoderB2);

  if (stateB1 == stateB2) {
    countB++;
  } else {
    countB--;
  }
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



