#include <Servo.h>

/*
 * Meachatronics 2 Project 2015: Putt Putt
 * Brandon Roberts
 * James Pace
 */

 // Pin Number Mapping
 #define FLEX_PIN 18   //Interrupt
 #define IR1      19   //Interrupt
 #define IR2      20  //Interrupt
 #define IR3      21  //Interrupt
 #define BRIDGE   2   //PWM
 #define WALL1    3   //PWM
 #define WALL2    4   //PWM

 //Servo Position Constants
 #define BRIDGEUP 0
 #define BRIDGEDOWN 90
 

 // Function protoypes for interrupt handlers.
 void flex_handler();
 void ir1_handler();
 void ir2_handler();

 // Global Variables.
 // (Can I get rid of some of these?)
 volatile int score = 0;
 volatile bool bridge_toggle = 0;
 volatile bool bridge_down = 0;
 volatile long bridge_down_time = 0;
 volatile long wall_swap_time = 0; 

 // Servos
 Servo bridge_servo;
 Servo wall1_servo;
 Servo wall2_servo;

void setup() {
  // put your setup code here, to run once:

  // Attach interrupt pins. Seemingly, marks as input for you...
  attachInterrupt(digitalPinToInterrupt(FLEX_PIN), flex_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR1), ir1_handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR2), ir2_handler, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR3), ir3_handler, FALLING);

  // Servos
  bridge_servo.attach(BRIDGE);
  wall1_servo.attach(WALL1);
  wall2_servo.attach(WALL2);

  // Put servos in initial position.
  bridge_servo.write(BRIDGEUP);
  wall1_servo.write(0);
  wall2_servo.write(90);

  // Set up serial monitor.
  Serial.begin(9600);

  // Reset initial status of globals. Doens't seem to reset otherwise...
  // Probably because they are volatile?
  score = 0;
  bridge_down = 0;
  bridge_down_time = 0;
  wall_swap_time = millis();
  bridge_toggle = 0; 
}

void loop() {
  // put your main code here, to run repeatedly:
   
   if(!bridge_down && bridge_toggle) {
    bridge_servo.write(BRIDGEDOWN);
    Serial.write("Trigger\n");
    bridge_down = !bridge_down;
    bridge_down_time = millis();
    bridge_toggle = 0;
   }

  if(((millis() - bridge_down_time) > 2000) && (bridge_down))
  {
    bridge_servo.write(BRIDGEUP);
    bridge_down = !bridge_down;
  }

  if((millis() - wall_swap_time) > 2000){
    if(wall1_servo.read() == 90){
      wall1_servo.write(0);
      wall2_servo.write(90);
    }else{
      wall1_servo.write(90);
      wall2_servo.write(0);
    }
    wall_swap_time = millis();
  }
  
  if(!(millis()%2000)){
    Serial.println(score);
  }

}

void flex_handler() {
  // When flex sensor is triggered, lower bridge, but only once.
  bridge_toggle = 1;
}

void ir1_handler() {
  // When IR1 is triggered, increase score by 1
  score = score + 1;
}

void ir2_handler(){
  // When IR2 is triggered, increase score by 2
  score = score + 2; 
}

void ir3_handler(){
  // When IR3 is triggered increase score by 3
  score = score + 3;
}

