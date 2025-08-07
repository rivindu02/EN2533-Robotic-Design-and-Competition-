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
// PID parameters
float Kp = 2.0;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 1.0;  // Derivative gain

// Variables
//volatile int encoderCountA = 0;    // Encoder counts for left motor
//volatile int encoderCountB = 0;   // Encoder counts for right motor
int targetSpeed = 100;           // Target speed (encoder counts per time step)
float error, lastError = 0;
float integral = 0;
float derivative = 0;
float correction;


int increment_count=0;

// Timer variables
unsigned long lastTime = 0;
unsigned long interval = 100; // 100 ms control loop interval

int lfspeed = 85;

// for Line Navigation

///// Encoders (only use 2, 3, 18, 19, 20, and 21)
// #define encoderA1 18     // Encoder 1 signal of motor A
// #define encoderA2 19     // Encoder 2 signal of motor A
// #define encoderB1 20     // Encoder 1 signal of motor A
// #define encoderB2 21     // Encoder 2 signal of motor A

#define ENCODER_A1 18       // Encoder A channel (Interrupt pin)
#define ENCODER_A2 19
#define ENCODER_B1 20     // Encoder B channel (Interrupt pin)
#define ENCODER_B2 21

volatile int encoderCountA = 0;  // Encoder count
volatile int encoderCountB = 0;
volatile int encoderCount_A = 0;  // Encoder count
volatile int encoderCount_B = 0;


// volatile bool lastA1State = LOW;
// volatile bool lastA2State = LOW;

// volatile bool lastB1State = LOW;
// volatile bool lastB2State = LOW;

L298N motor1(PWMA, AIN1, AIN2);     // left motor
L298N motor2(PWMB, BIN1, BIN2);


// for Line Navigation
int No_lines=0;

void setup() {
  
    for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);



  Serial.begin(9600);

  // Motor Encoder Pins
  pinMode(ENCODER_A1, INPUT_PULLUP);     // for pid encoders
  //pinMode(encoderA2, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  //pinMode(encoderB2, INPUT_PULLUP);



  pinMode(ENCODER_A2, INPUT_PULLUP);     // for lineNavigation

  pinMode(ENCODER_B2, INPUT_PULLUP);


  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR_A2, RISING);        // underscore
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), encoderISR_B2, RISING);
}

void loop() {




  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA);
  //Serial.print("\t");
  //Serial.println(encoderCountB);
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
  //delay(50);
  LineNavigation();
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
}


void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 135 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
  }
}

void readSensors_white(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 400 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
  }
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
    //Serial.print("Error: ");
    //Serial.print(error);
    //Serial.print(" Correction: ");
    //Serial.println(correction);
  }
}




void LineNavigation(){
  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA);
  //Serial.print("\t");
  //Serial.println(encoderCountB);

  //delay(100);  // Update every 100 ms for get the count

  // motor1.setSpeed(95);
  // motor1.forward();
  // motor2.setSpeed(110);
  // motor2.forward();        // instead of this use go_straight

  go_straight();

  // for white-->0
  // for black-->1

  // for minimize the Error get the sum and find whether it is greater than 4 for less than 4

  // have to get count roughly for 3cm and 6cm

  bool Code[20]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

  int encoder_pulse_3cm=40;
  int encoder_pulse_6cm=80;
  float threshold =(encoder_pulse_3cm + encoder_pulse_6cm)/2;

  readSensors_white(sensorValues);
  bool current_line= Identify_Strip(sensorValues); //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
  
  //Serial.print("current_line \t");
  //Serial.println(current_line);


  delay(5);
  readSensors_white(sensorValues);
  bool new_line=Identify_Strip(sensorValues);  //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7] ;

  //Serial.print("new_line \t");
  //Serial.println(new_line);

  // prev condition current_line=!new_line
  while (current_line==1 && new_line==0){ //enter into white strip
    No_lines++;

    Serial.print("No_lines: ");
    Serial.print(No_lines);
    Serial.println();

    //Serial.println(No_lines);
    encoderCount_A=0;
    encoderCount_B=0;
    while (new_line==0){
      delay(1);
      readSensors_white(sensorValues);
      new_line=Identify_Strip(sensorValues); //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
    }
    int encoder_pulse=(encoderCount_A+encoderCount_B)/2;
    
    //motor1.stop();
    //motor2.stop();
    //delay(500);

    Serial.print("encoder_pulse: ");
    Serial.print(encoder_pulse);
    Serial.println();





    if (encoder_pulse>threshold){
      Code[No_lines-1]=1;
    } else{
      Code[No_lines-1]=0;
    }

    //delay(200);    
  }

  if (No_lines>3){
    if (Code[No_lines-1]==0 && Code[No_lines-2]==0 && Code[No_lines-3]==0){
      
      bool Final_Code[No_lines-3];
      // number of lines in Final Code=(No_lines-3)

      for (int i=0; i<No_lines-3; i++){     // 0,1,2,3,.....,(No_lines-4) =====> // Final_Code lenght is (No_lines-3) 
        Final_Code[i]=Code[i];
      }
      
      //computeModulus(int* code);
      increment_count++;
      
      
      //Serial.println(Final_Code[No_lines]);

    }
  }


}
//   for 0 its black
//calibrate sensorValues 5 or more

bool Identify_Strip(int* sensorValues){
  if ((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7])>5){
    return 0;     
    
  }else{
    return 1;
  }
}


//in Identify_Strip----->    Black==> 1    White==> 0

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  motor1.setSpeed(leftSpeed);
  motor1.forward();

  motor2.setSpeed(rightSpeed);
  motor2.forward();

}

// Interrupt Service Routine (ISR) for encoder
void encoderISR_A1() {
  encoderCountA++;
}

void encoderISR_A2() {
  encoderCount_A++;
}


// Interrupt Service Routine (ISR) for Motor B encoder
void encoderISR_B1() {
  encoderCountB++;
}

void encoderISR_B2() {
  encoderCount_B++;
}



