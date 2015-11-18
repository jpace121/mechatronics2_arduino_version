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
   PRINT_SPEED  // print speed in degrees/s
*/
#define PRINT_SEPARATOR 1
#define PRINT_ENCODER   0
#define PRINT_DEGREES   0
#define PRINT_SPEED     1

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
 #define MOTOREN  A15  // GPIO
 #define MOTORC   A14  //GPIO
 #define MOTORD   A13  //GPIO
 // 5 and 6 skipped because both come from Timer0, which I'm using for the
 // timer compare already, and I don't want to overuse it if I can help it.

 //PID Constants
 #define CNTSPERREV 600. //estimate
 #define KP (1)
 #define KD (1)
 #define KI (1)
 #define DESIREDHZ (1.)

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
 unsigned int encoder_cnt = 0;
 int encode_speed = 0;
 unsigned int last_cnt_forspeed = 0; //this could probably be made a static in loop?

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

  // GPIO for enable and direction keys for windmill
  pinMode(MOTOREN, OUTPUT);
  pinMode(MOTORC, OUTPUT);
  pinMode(MOTORD, OUTPUT);

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
                         // CTC1 bit in TCCR1B
  
  // Reset initial status of globals. Doesn't seem to reset otherwise...
  // Probably because they are volatile?
  // (Are these needed? They don't hurt anything...)
  score = 0;
  bridge_down = 0;
  bridge_down_time = 0;
  wall_swap_time = millis();
  bridge_toggle = 0;

  // Test for lab.
  digitalWrite(MOTOREN, HIGH);
  digitalWrite(MOTORC, HIGH);
  digitalWrite(MOTORD, LOW);
  analogWrite(MOT_PWM,255);
  
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
      //I would rather calculate this stuff in an RTI, but my RTI is so small, I only see like two cnts ever.
      Serial.println(degreesFromCnts(encoder_cnt) - degreesFromCnts(last_cnt_forspeed)/1.);
      last_cnt_forspeed = encoder_cnt;
    #endif
    last_tx = millis();
  }

}



ISR(TIMER0_COMPA_vect) {
    // Interrupt Service Routine for the output compare.
    // Runs every 1.024 ms.
    // This may be too fast to be any good.
    // Interrupt will be used to change the PWM via PID.
    // For this function, I'm tracking everything in Hz.

    static double summed_error;
    static unsigned double last_cnt;
    int deriv = encoder_cnt - last_cnt; // This will be a really small number.

    summed_error += 


    last_cnt = encoder_cnt;

}

 double degreesFromCnts(unsigned int cnt) {
    return (double) ((cnt)*(360./CNTSPERREV));
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

