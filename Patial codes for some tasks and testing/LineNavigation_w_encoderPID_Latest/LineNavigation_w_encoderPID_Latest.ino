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
// #define encoderA1 18     // Encoder 1 signal of motor A
// #define encoderA2 19     // Encoder 2 signal of motor A
// #define encoderB1 20     // Encoder 1 signal of motor B
// #define encoderB2 21     // Encoder 2 signal of motor B

#define ENCODER_A2 19       // Encoder A channel (Interrupt pin)--->LineNavigation

#define ENCODER_B2 21     // Encoder B channel (Interrupt pin)--->LineNavigation no use

#define ENCODER_A1 18       // Encoder A channel (Interrupt pin)===> left----> encoder PID

#define ENCODER_B1 20     // Encoder B channel (Interrupt pin)=====> right---> encoder PID

volatile int encoderCountA2 = 0;  // Encoder count
volatile int encoderCountB2 = 0;

// Encoder counts
volatile int encoderCountA1 = 0;  // Encoder count for Motor A
volatile int encoderCountB1 = 0;  // Encoder count for Motor B
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

// volatile bool lastA1State = LOW;
// volatile bool lastA2State = LOW;

// volatile bool lastB1State = LOW;
// volatile bool lastB2State = LOW;

L298N motor1(PWMA, AIN1, AIN2);     // left motor
L298N motor2(PWMB, BIN1, BIN2);


// for Line Navigation
int No_lines=0;

void setup() {

    // Setup motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);


  // Setup encoder pins
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  
    for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }




  Serial.begin(9600);

  // Motor Encoder Pins
  pinMode(ENCODER_A2, INPUT_PULLUP);     // don't know whether its pullup or pulldown
  //pinMode(encoderA2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  //pinMode(encoderB2, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), encoderISR_B, RISING);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), countRight, RISING);


}

void loop() {
  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA2 2);
  //Serial.print("\t");
  //Serial.println(encoderCountB2);
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
  //delay(50);
  LineNavigation();
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
  //moveStraightPID();
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


void LineNavigation(){
  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA2);
  //Serial.print("\t");
  //Serial.println(encoderCountB2);

  //delay(100);  // Update every 100 ms for get the count


  // motor1.setSpeed(90);
  // motor1.forward();
  // motor2.setSpeed(110);
  // motor2.forward();



  moveStraightPID();
  // for white-->0
  // for black-->1

  // for minimize the Error get the sum and find whether it is greater than 4 for less than 4

  // have to get count roughly for 3cm and 6cm

  bool Code[20]={0};

  int encoder_pulse_3cm=40;
  int encoder_pulse_6cm=80;
  float threshold =(encoder_pulse_3cm + encoder_pulse_6cm)/2;

  readSensors_white(sensorValues);
  bool current_line= Identify_Strip(sensorValues); //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
  
  //Serial.print("current_line \t");
  //Serial.println(current_line);


  delay(1);
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
    int encoderCountA3=0;
    encoderCountA3=encoderCountA2;
    
    
    //encoderCountB2=0;     //before
    while (new_line==0){
      delay(1);
      readSensors_white(sensorValues);
      new_line=Identify_Strip(sensorValues); //sensorValues[0]  && sensorValues[1]  && sensorValues[2]  && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7];
    }
    //int encoder_pulse=encoderCountA2;   // (encoderCountA2+encoderCountB2)/2;     // befooree
    
    int encoder_pulse=encoderCountA2-encoderCountA3;

    // motor1.stop();
    // motor2.stop();
    // delay(500);

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
      
      bool Final_Code[No_lines];
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





// Interrupt Service Routine (ISR) for encoder
void encoderISR_A() {
  encoderCountA2++;
}

// Interrupt Service Routine (ISR) for Motor B encoder
void encoderISR_B() {
  encoderCountB2++;
}


// Interrupt service routines for encoders
void countLeft() {
  encoderCountA1++;
}

void countRight() {
  encoderCountB1++;
}





