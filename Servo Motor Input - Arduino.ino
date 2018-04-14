
#include <Servo.h>

Servo ultrasonic_servo; 
const int Rpi_input180 = 10; //select arduino input port
const int Rpi_inputLeft = 11;
const int Rpi_inputRight = 12;
const int Rpi_input0 = 9;
int Input_State_180 = 0;
int Input_State_Left = 0;
int Input_State_Right = 0;
int Input_State_0 = 0;

void setup() {
  pinMode(Rpi_input180,INPUT);
  pinMode(Rpi_inputLeft,INPUT);
  pinMode(Rpi_inputRight,INPUT);
  pinMode(Rpi_input0,INPUT);
  ultrasonic_servo.attach(3); //select servo output port
}

void loop() {
  Input_State_180 = digitalRead(Rpi_input180);
  Input_State_Left = digitalRead(Rpi_inputLeft);
  Input_State_Right = digitalRead(Rpi_inputRight);
  Input_State_0 = digitalRead(Rpi_input0);
  
  if (Input_State_180 == HIGH)
    ultrasonic_servo.write(180);

  else if (Input_State_Left == HIGH)
    ultrasonic_servo.write(153);

  else if (Input_State_Right == HIGH)
    ultrasonic_servo.write(20);

  else if (Input_State_0 == HIGH)
    ultrasonic_servo.write(0);
    
  else
    ultrasonic_servo.write(82);
     
}
