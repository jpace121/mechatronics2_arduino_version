#include <Servo.h>

/*
 * Meachatronics 2 Project 2015: Putt Putt
 * Brandon Roberts
 * James Pace
 */

// Debug definitions.
/* Options that the program looks for (1 is on 0 is off):
   PRINT_SEPARATOR // print line to separator each debug block
   PRINT_ENCODER // print encoder cnts every few seconds
   PRINT_DEGREES // print position of motor every few seconds
   PRINT_SPEED  // print speed in Hz
   PRINT_DIFF  // print encoder count difference
   PRINT_PWM // print pwm from PID
   PRINT_SEVSEG // print values written to Port L
   PRINT_SCORE // print score
 */
#define PRINT_SEPARATOR 1
#define PRINT_ENCODER   0
#define PRINT_DEGREES   0
#define PRINT_SPEED     0
#define PRINT_DIFF      0
#define PRINT_PWM       0
#define PRINT_SEVSEG    1
#define PRINT_SCORE     1

 // Pin Number Mapping (PINOUT)
 #define FLEX_PIN 18   //Interrupt
 #define IR1      19   //Interrupt
 #define IR2      20  //Interrupt
 #define IR3      21  //Interrupt
 #define WALL1    9   //PWM 
 #define WALL2    8   //PWM
 #define BRIDGE   7   //PWM
 #define MOT_PWM  10   //PWM
 #define ENCODEA  3   //Interrupt
 #define ENCODEB  2   //Interrupt
 #define MOTORC   A14  //GPIO
 #define MOTORD   A13  //GPIO
 #define SEVSEG   PORTL //GPIO Pins 35-42 for Seven Seg
 // 13 and 4 skipped because both come from Timer0, used by millis.
 // 12 and 11 skipped becaused used by Timer 1, which causes the PWM to be
 // distorted.
 // Source: http://playground.arduino.cc/Main/TimerPWMCheatsheet

//PID Constants
#define CNTSPERREV (600.) //estimate
#define SAMPLETIME (0.032) //s
#define KP (20.)
#define KD (7.)
#define KI (2.)
#define DESIREDHZ (3.)

 //Definitions for AVR pin setting functions.
 #define sbi(port,bit) \
    (port) |= (1 << (bit))
 #define cbi(port,bit) \
    (port) &= ~(1 << (bit)) 

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
int score = 0;
 bool bridge_toggle = 0;
 bool bridge_down = 0;
 long bridge_down_time = 0;
 long wall_swap_time = 0;
 long last_tx = 0;
 volatile unsigned int encoder_cnt = 0;
 volatile double encode_speed = 0.;
 volatile unsigned int last_cnt_forspeed = 0;
 volatile int encode_diff = 0;
 volatile int summederror = 0;
 volatile int newPWM = 0;

 // Servos
 Servo bridge_servo;
 Servo wall1_servo;
 Servo wall2_servo;

// Macros for 7 Seg.
#define print7seg(score) \
    ((((score)/10)<<4) | ((score)%10)) 

void setup() {
  // put your setup code here, to run once:

  // Attach interrupt pins. Seemingly, marks as input for you...
  attachInterrupt(digitalPinToInterrupt(FLEX_PIN), flex_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR1), ir1_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR2), ir2_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR3), ir3_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODEA), encoderA_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODEB), encoderB_handler, RISING);

  // GPIO for enable and direction keys for windmill
  pinMode(MOTORC, OUTPUT);
  pinMode(MOTORD, OUTPUT);
  pinMode(MOT_PWM, OUTPUT);

  pinMode(13, OUTPUT); // this is the bultin LED

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
  // Turn on Timer 1
  cli(); //disable interrupts
  TCCR1A = 0; // This is important!!
  TCCR1B = 0; // This is important!!
  cbi(PRR0, PRTIM1);
  // Select clock/8 which is about 2 MHz or 0.5*10-6 s.
  sbi(TCCR1B, CS11);
  // Set interrupt on overflow of timer 1
  sbi(TIMSK1, TOIE1);
  sei(); //reenable interrupts
  
  // Reset initial status of globals. Doesn't seem to reset otherwise...
  // Probably because they are volatile?
  // (Are these needed? They don't hurt anything...)
  //score = 0;
  bridge_down = 0;
  bridge_down_time = 0;
  wall_swap_time = millis();
  bridge_toggle = 0;

  // Seven Segment Display Set Up
  DDRL = 0xff; // All of PortL is output is output.

  // Test for lab.
  analogWrite(MOT_PWM, 255);
  digitalWrite(MOTORC, HIGH);
  digitalWrite(MOTORD, LOW);
  
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

  // Debug serial prints, once every 1 seconds.
  if((millis()-last_tx) > 1000){
    #if PRINT_SEPARATOR
      Serial.flush();
      Serial.println("------------");
    #endif
    #if PRINT_ENCODER
      Serial.print("Encoder Count: ");
      Serial.println(encoder_cnt);
    #endif
    #if PRINT_DEGREES
      Serial.print("Position in Degrees: ");
      Serial.println(degreesFromCnts(encoder_cnt));
    #endif
    #if PRINT_SPEED
      Serial.print("Speed: ");
      Serial.println(encode_speed);
    #endif
    #if PRINT_DIFF
      Serial.print("Diff: ");
      Serial.println(encode_diff);
    #endif
    #if PRINT_PWM
       Serial.print("PWM Out: ");
       Serial.println(newPWM);
    #endif
    #if PRINT_SEVSEG
       Serial.print("Sev Seg Port: ");
       Serial.println(SEVSEG);
    #endif   
    #if PRINT_SCORE
       Serial.print("Score: ");
       Serial.println(score);
    #endif   
    last_tx = millis();
  }

  SEVSEG = print7seg(score);
}



ISR(TIMER1_OVF_vect) {
    // Interrupt Service Routine for the output compare.
    // Interrupt will be used to change the PWM via PID.
    // For this function, I'm tracking everything in Hz.

    // Calculate Speed.
    encode_speed = ((encoder_cnt - last_cnt_forspeed)*2*PI)/(CNTSPERREV*SAMPLETIME);
    encode_diff = encoder_cnt - last_cnt_forspeed;
    last_cnt_forspeed = encoder_cnt;

    // PID Controller for speed.
    // I think this only will work is moving in one direction.
    summederror += (DESIREDHZ - encode_speed);
    if (DESIREDHZ == encode_speed) { // Antiwindup.
        summederror = 0;
    }
    newPWM = KP*(DESIREDHZ-encode_speed) + KD*encode_diff + KI*summederror;
    /*Limit checks. */
    if(newPWM > 255) { 
        newPWM = 255;
    }
    if(newPWM < 0) {
        newPWM = 0;
    }

    analogWrite(MOT_PWM, newPWM); // Actually write stuff.
    // TODO: Add direction stuff.

    digitalWrite(13, !digitalRead(13));

}

 double degreesFromCnts(unsigned int cnt) {
    return (double) ((cnt)*(360./CNTSPERREV));
 }

double radFromCnts(unsigned int cnt) {
    return (double) ((cnt)*(2*PI/CNTSPERREV));
 }

/*
 Encoder stuff stolen from:
     http://www.edn.com/design/integrated-circuit-design/4363949/Decode-a-quadrature-encoder-in-software
 */

void encoderA_handler() {
    // Triggers on rise of encoder channel A.
    // If here, then A is high, and we should check B. If B low, than A is leading.
    // A leading will be direction positive.
    if(digitalRead(ENCODEB) == 1) {
        encoder_cnt--;
    } else {
        encoder_cnt++;
    }
}

void encoderB_handler() {
    // Triggers on rise of encoder channel B.
    // If here, then B is high, and we should check A. If A low, than B is leading.
    // A leading will be direction positive.
    if(digitalRead(ENCODEA) == 1) {
        encoder_cnt++;
    } else {
        encoder_cnt--;
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

