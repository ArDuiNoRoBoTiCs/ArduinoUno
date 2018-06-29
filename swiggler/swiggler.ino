

/**
 * These are the individual pins that are connected from the Arduino to the "L298n module", 
 * which controls the 4 wheel motors. For example, I/O pin 5 is hard-wired to the ENA pin 
 * on the L298n, and pin 6 is connected to the L298n ENB pin.
 * 
 * Here's how these pins work:
 * 
 *  - You cannot individually control the left-front and left-rear motors (nor the right-front 
 *   and right-rear motors); You can atomically control "the left side" and "the right side".
 * 
 * ENA enables the 2 left motors. You can set it to any value from 0 to 255
 *   to control the speed of those motors; 0 if off, 255 is maximum RPM. 128 would be half speed.
 *   [Uses 8-bit Pulse Width Modulation; see https://en.wikipedia.org/wiki/Arduino_Uno]
 *   
 * IN1 & IN2 work together to control the direction of the 2 left motors, which can have 4 
 *   different values: brake, coast, forward, and backward. When IN1 and IN2 are both set to LOW, it
 *   is set to coast; when both set to HI, the motors are braked. If IN1 is HI, and IN2 is LOW, 
 *   the motors spin forward. If IN1 is LOW, and IN2 is HI, the motors spin backward.
 *   [NOTE: looks like pin 9 and 11 are capable of 8-bit PWM, per https://en.wikipedia.org/wiki/Arduino_Uno;
 *   however, we are hard-wired to the L298n motor driver board.
 *   
 * ENB has the same effect on the 2 right motors as ENA has on the left.
 * 
 * IN3 & IN4 control the direction of the 2 right motors, similar to how IN1 & IN2 work for the left side.
 *   However, the directions are reversed; If IN3 is HI, and IN4 is LOW, the motors spin backward.
 *   If IN3 is LOW, and IN4 is HI, the motors spin forward.
 *   
 *   Here's the full table ot possibilities based on the values of 6 pins, from the Arduino UNO tutorial:
 *   
 *  ENA   ENB   IN1   IN2   IN3   IN4   Description  
 *  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
 *  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
 *  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
 *  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
 *  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stoped
 *  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stoped
 *  LOW   LOW   N/A   N/A   N/A   N/A   Car is stoped
 *  
 *  
 *  Also see https://en.wikipedia.org/wiki/Arduino_Uno
 */
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

/**
 * Set both sides to highest power.
 */
void high(){
  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH);
}

/**
 * Right motors forward.
 */
void forwardRight(){
  digitalWrite(IN3, LOW);  
  digitalWrite(IN4, HIGH);
}

/**
 * Left motors forward.
 */
void forwardLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
}

/**
 * Left motors backward.
 */
void backLeft(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
}

/**
 * Right motors backward.
 */
void backRight() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/**
 * Move car forward.
 */
void forward(){
  high();
  forwardLeft();
  forwardRight(); 
}

/**
 * Move car backward.
 */
void back(){
  high();
  backLeft();
  backRight();
}

/**
 * Turn car left.
 */
void left(){
  high();
  backLeft();
  forwardRight();
}

/**
 * Turn car right.
 */
void right(){
  high();
  forwardLeft();
  backRight();
}

/**
 * Stop the car.
 */
void stop(){
  Serial.println("stop");
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

/**
 * Makes the robot swiggle.
 * always runs for 200 millis
 */
void swiggle() {
  right();
  delay(100);
  left();
  delay(100);
}

/**
 * Will swiggle for the given number of milliseconds.
 * @param millis The number of milliseconds to swiggle. 
 */
void swiggleMilliseconds(int millis) {
  int numIterations = millis / 200;
  for (int n = 0 ; n < numIterations ; n++) {
     swiggle();
  }
}

/**
 * Will swiggle for the given number of seconds.
 * @paran secs The number of seconds to swiggle.
 */
void swiggleSeconds(int secs) {
  swiggleMilliseconds(secs * 1000);
}


void setup() {
  Serial.println("setup");
  swiggleSeconds(2);
  forward();
  delay(1000);
  left();
  delay(500);
  forward();
  delay(500);
  swiggleSeconds(2);
  back();
  delay(500);
  right();
  delay(500);
  back();
  delay(1000);
  forward();
  delay(1000);
  right();
  delay(500);
  forward();
  delay(500);
  swiggleSeconds(2);
  back();
  delay(500);
  left();
  delay(500);
  back();
  delay(1000);
  back();
  delay(1000);
  right();
  delay(500);
  back();
  delay(500);
  swiggleSeconds(2);
  forward();
  delay(500);
  left();
  delay(500);
  forward();
  delay(1000);
  back();
  delay(1000);
  left();
  delay(500);
  back();
  delay(500);
  swiggleSeconds(2);
  forward();
  delay(500);
  right();
  delay(500);
  forward();
  delay(1000);
  swiggleSeconds(2);
  stop();
}



void loop() {
  swiggle();
}
