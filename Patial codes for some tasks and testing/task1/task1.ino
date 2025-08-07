#include <L298N.h>
#include <math.h>

// A====> left
// B====> Right

#define PWMA 4
#define PWMB 9
#define AIN1 5
#define AIN2 6
#define BIN1 7
#define BIN2 8
// 23,25..29


// for IR_array
const int SensorCount = 8; // Total number of sensors
const int analogSensorCount = 8; // Number of analog sensors
const int analogSensorPins[analogSensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for the remaining sensors
int IR_weight[SensorCount] = {-50, -30, -15, -5, 5, 15, 30, 50};
float errorArray[50] = {0};
int sensorValues[SensorCount];

// PID control parameters(in IR-Array)
float Kp = 5 ; // Proportional term
float Ki = 0; // Integral term
float Kd = 5;// Derivative term
float sum;
int increment_count = 0;

float error;

// PID variables
float P, I, D, previousError = 0;
float lsp, rsp;


String junction = "a";



// for Line Navigation

///// Encoders (only use 2, 3, 18, 19, 20, and 21)


// for Line Navigation
//#define ENCODER_A2 19       // Encoder A channel (Interrupt pin)--->LineNavigation

//#define ENCODER_B2 21     // Encoder B channel (Interrupt pin)--->LineNavigation no use





#define ENCODER_A1 2       // Encoder A channel (Interrupt pin)===> left----> encoder PID

#define ENCODER_B1 3     // Encoder B channel (Interrupt pin)=====> right---> encoder PID



//for Line Navigation
//volatile int encoderCountA2 = 0;  // Encoder count
//volatile int encoderCountB2 = 0;

// Encoder counts
volatile long encoderCountA = 0;  // Encoder count for Motor A
volatile long encoderCountB = 0;  // Encoder count for Motor B
const long targetCounts = 190;   // Desired encoder counts for forward movement



// PID parameters for encoders
//float Kp = 5;  // Proportional gain
//float Ki = 0;  // Integral gain
//float Kd = 5;  // Derivative gain

// PID variables
float errorenco = 0;
float previousErrorenco = 0;
float integralenco = 0;
float derivativeenco = 0;

int baseSpeed=130;    // previous 110

// volatile bool lastA1State = LOW;
// volatile bool lastA2State = LOW;

// volatile bool lastB1State = LOW;
// volatile bool lastB2State = LOW;

L298N motor1(PWMA, AIN1, AIN2);     // left motor
L298N motor2(PWMB, BIN1, BIN2);


// for Line Navigation
int No_lines=0;

int modulus;


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
  // 

  // for line navigation
  //pinMode(ENCODER_A2, INPUT_PULLUP);     // don't know whether its pullup or pulldown

  //pinMode(ENCODER_B2, INPUT_PULLUP);


  // Attach interrupts for encoders for line Navigation
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoderISR_A, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_B2), encoderISR_B, RISING);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);


}

void loop() {
  //Serial.print("Pulse Count: ");
  //Serial.print(encoderCountA2 2);
  //Serial.print("\t");
  //Serial.println(encoderCountB2);
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
  //delay(50);
  //LineNavigation();
  //readSensors(sensorValues);
  //Serial.println(Identify_Strip(sensorValues));
  //moveStraightPID();
  //readSensors_debug(sensorValues);
  
  LineNavigation();


    
  // }
}


void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > 80 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
  }
}



// void readSensors_debug(int *values) {
//   // Read analog sensors
//   for (int i = 0; i < analogSensorCount; i++) {
//     values[i] = analogRead(analogSensorPins[i]) > 80 ? 0 : 1; // Assuming higher values indicate no line          (sensor readings > 135) white ====> return 0 (on line)
//     Serial.print(values[i]);
//     Serial.print("\t");
  
//   }

// }


bool Code[20]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int pulses_array[20]={0};
int first_black_line=0;
//int True_Lines=0;


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
//   for 0 its black
//calibrate sensorValues 5 or more

bool Identify_Strip(int* sensorValues){
  readSensors(sensorValues);
  if ((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7])>5){
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 1;     // White line
    
  }else{
    //Serial.println((sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7]));
    return 0;
  }
}


//////////////in Identify_Strip----->    Black==> 1    White==> 0


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



// encoderISR_A ,encoderISR_B for Line Navigation

// Interrupt Service Routine (ISR) for encoder   
//void encoderISR_A() {
//  encoderCountA2++;
//}

// Interrupt Service Routine (ISR) for Motor B encoder
//void encoderISR_B() {
//  encoderCountB2++;
//}


// Interrupt service routines for encoders
void encoderISR_A() {
  encoderCountA++;
}

void encoderISR_B() {
  encoderCountB++;
}