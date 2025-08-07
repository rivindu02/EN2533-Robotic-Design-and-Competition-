#include <Servo.h> // Include the Servo library

Servo servo_grabbing; // Create a servo object
Servo servo_lifting;

void setup() {
  servo_grabbing.attach(11); // Attach the servo to pin 9
  servo_lifting.attach(12);
}

void loop() {
  boxlift();
  boxdrop();

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


























