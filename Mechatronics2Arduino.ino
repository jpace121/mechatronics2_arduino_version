#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
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
   PRINT_SCORE // print score
   PRINT_PCINT //print which PCINT triggered
 */
#define PRINT_SEPARATOR 1
#define PRINT_ENCODER   0
#define PRINT_DEGREES   0
#define PRINT_SPEED     1
#define PRINT_DIFF      1
#define PRINT_PWM       1
#define PRINT_SCORE     0
#define PRINT_PCINT     0

 // Pin Number Mapping (PINOUT)
 #define FLEX_PIN 18   //Interrupt
 #define IR1     (1<<5)  //11   //PB5 Interrupt
 #define IR2     (1<<6)  //12  //PB6 Interrupt 
 #define IR3     (1<<7)  //13  //PB7 Interrupt
 #define WALL1    9   //PWM 
 #define WALL2    8   //PWM
 #define BRIDGE   7   //PWM
 #define MOT_PWM  10   //PWM
 #define ENCODEA  2   //Interrupt
 #define ENCODEB  3   //Interrupt
 #define MOTORC   A14  //GPIO
 #define MOTORD   A13  //GPIO
#define i2c_CLK  21   //i2c
#define i2c_DAT  20 //i2c
 // 13 and 4 skipped because both come from Timer0, used by millis.
 // 12 and 11 skipped becaused used by Timer 1, which causes the PWM to be
 // distorted.
 // Source: http://playground.arduino.cc/Main/TimerPWMCheatsheet

//PID Constants
#define CNTSPERREV (600.) //estimate
#define SAMPLETIME (0.032) //s
#define KP (45.)
#define KD (15.)
#define KI (30.)
#define DESIREDHZ (1.)

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

// Alphanumeric displays
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();

// Macros for 7 Seg.
#define print7seg(score) \
    ((((score)/10)<<4) | ((score)%10)) 

void setup() {
  // put your setup code here, to run once:

  // Attach interrupt pins. Seemingly, marks as input for you...
  attachInterrupt(digitalPinToInterrupt(FLEX_PIN), flex_handler, RISING);
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

  // set up pc interrupt for scoring, which frees the i2c pin.
  DDRB &= ~((1 << 7) | (1 << 6) | (1 << 5)); // mark as input (0)
  sbi(PCICR,PCIE0); // enable pcinterrupts for Port B pins (50-53, 10-13)
  PCMSK0 = 0; // clear mask
  sbi(PCMSK0,PCINT5); //turn on for PB5
  sbi(PCMSK0,PCINT6); //turn on for PB6
  sbi(PCMSK0,PCINT7); //turn on for PB7
  
  // Reset initial status of globals. Doesn't seem to reset otherwise...
  // Probably because they are volatile?
  // (Are these needed? They don't hurt anything...)
  score = 0;
  bridge_down = 0;
  bridge_down_time = 0;
  wall_swap_time = millis();
  bridge_toggle = 0;


  // Test for lab.
  analogWrite(MOT_PWM, 255);
  digitalWrite(MOTORC, HIGH);
  digitalWrite(MOTORD, LOW);

  // Set up alphanumeric display.
  alpha4.begin(0x70);
  alpha4.writeDigitAscii(0,'0');
  alpha4.writeDigitAscii(1,'0');
  alpha4.writeDigitAscii(2,'0');
  alpha4.writeDigitAscii(3,'0');
  alpha4.writeDisplay();
  
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
    #if PRINT_SCORE
       Serial.print("Score: ");
       Serial.println(score);
    #endif   
    last_tx = millis();
  }

  scoreToDisplay(score);

}



ISR(TIMER1_OVF_vect) {
    // Interrupt Service Routine for the output compare.
    // Interrupt will be used to change the PWM via PID.
    // For this function, I'm tracking everything in Hz.

    int err = (1*2*PI)/(CNTSPERREV*SAMPLETIME); //allow us to be off by one count

    // Calculate Speed.
    encode_speed = ((encoder_cnt - last_cnt_forspeed)*2*PI)/(CNTSPERREV*SAMPLETIME);
    encode_diff = encoder_cnt - last_cnt_forspeed;
    last_cnt_forspeed = encoder_cnt;

    // PID Controller for speed.
    // I think this only will work is moving in one direction.
    summederror += (DESIREDHZ - encode_speed);
    // Adjustment to the provided anti-windup code. We never exactly hit our Hz value,
    // so use and error value to be close-ish.
    /*if ((encode_speed > (DESIREDHZ - err)) && (encode_speed < (DESIREDHZ + err))){
        summederror = 0;
        }*/
    newPWM = KP*(DESIREDHZ-encode_speed) + KD*encode_diff + KI*summederror;
    /*Limit checks. */
    if(newPWM > 255) { 
        newPWM = 255;
        summederror -= (DESIREDHZ - encode_speed); //anti-windup
    }
    if(newPWM < -255) {
        newPWM = -255;
        summederror -= (DESIREDHZ - encode_speed); //anti-windup
    }
    if(newPWM < 0) {
        // Set direction opposite.
        digitalWrite(MOTORC, LOW);
        digitalWrite(MOTORD, HIGH);
        analogWrite(MOT_PWM, -1*newPWM); // Actually write stuff, with change in direction.
    }
    if( newPWM > 0) {
        // Set direction same as now.
        digitalWrite(MOTORC, HIGH);
        digitalWrite(MOTORD, LOW);
        analogWrite(MOT_PWM, newPWM); // Actually write stuff.
    }

    digitalWrite(13, !digitalRead(13));

}

double degreesFromCnts(unsigned int cnt) {
    return (double) ((cnt)*(360./CNTSPERREV));
 }

double radFromCnts(unsigned int cnt) {
    return (double) ((cnt)*(2*PI/CNTSPERREV));
 }

void scoreToDisplay(int val) {
    if(val > 9999) { // check for overflow
        val = 0;
    }
    // I don't liek how this changes val...
    int thous = val/1000;
    val = val%1000;
    int hunds = val/100;
    val = val % 100;
    int tens = val / 10;
    int ones = val % 10;
    alpha4.writeDigitAscii(0,(char)(thous+48));
    alpha4.writeDigitAscii(1,(char)(hunds+48));
    alpha4.writeDigitAscii(2,(char)(tens+48));
    alpha4.writeDigitAscii(3,(char)(ones+48));
    alpha4.writeDisplay();
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

// IR sensor interrupt.
ISR(PCINT0_vect) {
    byte PinState = PINB & 0b11100000; // mask to remove accidental
                                    // reads of other pins
    static byte lastPin;

    /*
      By making these else-if, can guarantee all won't go off on first interrupt
      if something is not correctly plugged in.
     */
    if((PinState & IR1) && ((lastPin & IR1) != (PinState & IR1))) {
        #if PRINT_PCINT
            Serial.println("IR1");
        #endif
        score += 1;
    }
    else if((PinState & IR2) && ((lastPin & IR2) != (PinState & IR2))) {
        #if PRINT_PCINT
            Serial.println("IR2");
        #endif
        score += 2;
    }
    else if((PinState & IR3) && ((lastPin & IR3) != (PinState & IR3))) {
        #if PRINT_PCINT
            Serial.println("IR3");
        #endif
        score += 3;
    }

    lastPin = PinState;
}

