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
 #define WALL1    9   //PWM 
 #define WALL2    8   //PWM
 #define BRIDGE   7   //PWM
 #define MOT_PWM  4   //PWM
 #define ENCODEA  3   //Interrupt
 #define ENCODEB  2   //Interrupt
 // 5 and 6 skipped because both come from Timer0, which I'm using for the
 // timer compare already, and I don't want to overuse it if I can help it.

 //Servo Position Constants
 #define BRIDGEUP 0
 #define BRIDGEDOWN 90

 // Function protoypes for interrupt handlers.
 void flex_handler();
 void ir1_handler();
 void ir2_handler();
 void encoderA_handler();
 void encoderB_handler();

 // Global Variables.
 // (Can I get rid of some of these by using statics?)
 volatile int score = 0;
 volatile bool bridge_toggle = 0;
 volatile bool bridge_down = 0;
 volatile long bridge_down_time = 0;
 volatile long wall_swap_time = 0;
 volatile long last_tx = 0;
 volatile int encoderA_cnt = 0;
 volatile int encoderB_cnt = 0;

 // Servos
 Servo bridge_servo;
 Servo wall1_servo;
 Servo wall2_servo;

void setup() {
  // put your setup code here, to run once:

  // Attach interrupt pins. Seemingly, marks as input for you...
  attachInterrupt(digitalPinToInterrupt(FLEX_PIN), flex_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR1), ir1_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR2), ir2_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR3), ir3_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODEA), encoderA_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODEB), encoderB_handler, RISING);

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

  // Set up RTI handler.
  // Inspiration gleaned from:
  //     https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
  // Approach: use the fact that Timer0 is set up already for millis by setting
  // up an output compare to a random time and then running our interrupt code.
  OCR0A = 0xAF; // set time to trigger interrupt, exact time does not matter,
                // will hit any time once a cycle. Each cycle is approximately
                // 1.024 ms.
  TIMSK0 |= _BV(OCIE0A); // enables interrupt on output compare.
                         // BV is a macro for 1 << (x)
                         // OCIE0A is the position bit which is predefined for
                         // us.
  
  // Reset initial status of globals. Doesn't seem to reset otherwise...
  // Probably because they are volatile?
  // (Are these needed? They don't hurt anything...)
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
  
  if((millis()-last_tx) > 1000){
    Serial.println(score);
    last_tx = millis();
  }

}

ISR(TIMER0_COMPA_vect) {
    // Interrupt Service Routine for the output compare.
    // Runs every 1.024 ms.
    // Interrupt will be used to change the PWM via PID.
    static int last_cnt = 0;

    /*
      Algorithm:
      1. Find current speed and direction from encoder counts.
      2. Compare to desired speed.
      3. Calculate new control PWM using PID.
     */

    //Find current speed from encoder counts.
    
    

}

void encoderA_handler() {
    // Triggers on rise of encoder channel A.
    encoderA_cnt++;
}

void encoderB_handler() {
    // Triggers on rise of encoder channel B.
    encoderB_cnt++;
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

