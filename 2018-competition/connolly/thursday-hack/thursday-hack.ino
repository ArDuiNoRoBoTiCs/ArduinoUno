#include <Wire.h>
#include <Servo.h>
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <LiquidCrystal_I2C.h>

/**
 * Distance a wall needs to be (in centimeters) to pose a threat.
 */
#define CRTICAL_DISTANCE 20

/**
 * Starting car speed. Note that 80 and under results in a loud whining sound from the 
 * motors, and the wheels dont turn - not enough power.
 */
#define DEFAULT_CAR_SPEED 150

/**
 * Arduino I/O Pins, and what they are hooked up to.
 */
#define DO_NOT_USE_SERIAL_RECEIVE 0 // used by serial library
#define DO_NOT_USE_SERIAL_TRANSMIT 1 // used by serial library
#define LINETRACKER_LEFT 2
#define SERVO_CONTROL 3
#define LINETRACKER_MIDDLE 4
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define LINETRACKER_RIGHT 10
#define IN4 11
#define INFRARED_SENSOR 12
#define LED 13
#define AVAILABLE_A0 A0
#define AVAILABLE_A1 A1
#define ULTRASONIC_ECHO A2 // moved from A4
#define ULTRASONIC_TRIGGER A3 // moved from A5
#define DO_NOT_USE_SCL A4 // used by lcd library
#define DO_NOT_USE_SDTA A5 // used by lcd library

// *** LCD subsystem
// set the LCD address to 0x20 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// *** Collision Detection subsystem
int disableCollisionDetection = true; // HACK
Servo ultrasonicServo;
int rightDistance = 0;
int leftDistance = 0;
int middleDistance = 0;
int justWentBack = false;

// *** Line Detect subsystem
int sensorRight = 0;
int sensorMiddle = 0;
int sensorLeft = 0;

// *** Motor subsystem
int carSpeed = DEFAULT_CAR_SPEED;

/**
 * Takes over the fourth (last) row of the LCD.
 */
void lcdPrintMotorStatus(String mesg) {
  static const String label = "mtr: ";
  static const int rowNumber = 3;
  
  lcd.setCursor(0,rowNumber);
  lcd.print(label);
  lcd.setCursor(label.length(), rowNumber);

  String fmtMessage = mesg + " (" + String(carSpeed) + ")";
  lcd.print(fmtMessage);
  
  // Erase the rest of the line.
  if (fmtMessage.length() + label.length() < 20) {
    for (int i = 0 ; i < 20 - (fmtMessage.length() + label.length()) ; i++) {
      lcd.print(' ');
    }
  }

  // Report to the serial monitor, in case the laptop is hooked up.
  Serial.print("motor: ");
  Serial.println(fmtMessage);
}


/**
 * Takes over the third row of the LCD.
 */
void lcdPrintCollisionDetectStatus(String mesg) {
  static const String label = "cld: ";
  static const int rowNumber = 2;
  static unsigned long lastWriteTime = 0;
  static int lastWriteDistance = 0;

  // Only update the distance every half-second, or it looks too freaky.
  unsigned long now = millis();
  if (now - lastWriteTime > 125) {
    lastWriteDistance = middleDistance;
    lastWriteTime = now;
  }
  
  lcd.setCursor(0,rowNumber);
  lcd.print(label);
  lcd.setCursor(label.length(), rowNumber);

  String fmtMessage = mesg + " (" + String(lastWriteDistance) + ")";
  lcd.print(fmtMessage);

  // Erase the rest of the line.
  if (fmtMessage.length() + label.length() < 20) {
    for (int i = 0 ; i < 20 - (fmtMessage.length() + label.length()) ; i++) {
      lcd.print(' ');
    }
  }

  // TODO: special case, supress too many stops.
  // Report to the serial monitor, in case the laptop is hooked up.
  Serial.print("collisionDetect: ");
  Serial.println(fmtMessage);
}


void lineDetectReadSensor() {
  sensorRight = !digitalRead(LINETRACKER_RIGHT);
  sensorMiddle = !digitalRead(LINETRACKER_MIDDLE);
  sensorLeft = !digitalRead(LINETRACKER_LEFT);
}

/**
 * Takes over the second row of the LCD.
 */
void lcdPrintLineDetectStatus(String mesg) {
  static const String label = "lin: ";
  static const int rowNumber = 1;
  static unsigned long lastWriteTime = 0;
  static int lastWriteRight = 0;
  static int lastWriteMiddle = 0;
  static int lastWriteLeft = 0;
  
  lcd.setCursor(0,rowNumber);
  lcd.print(label);
  lcd.setCursor(label.length(), rowNumber);
  
  // Only update the distance every half-second, or it looks too freaky.
  unsigned long now = millis();
  if (now - lastWriteTime > 500) {
    lastWriteRight = sensorRight;
    lastWriteMiddle = sensorMiddle;
    lastWriteLeft = sensorLeft;
    lastWriteTime = now;
  }
  
  String fmtMessage = mesg + " (" + String(lastWriteLeft) + "," + String(lastWriteMiddle) + "," + String(lastWriteRight) + ")";
  lcd.print(fmtMessage);
  
  // Erase the rest of the line.
  if (fmtMessage.length() + label.length() < 20) {
    for (int i = 0 ; i < 20 - (fmtMessage.length() + label.length()) ; i++) {
      lcd.print(' ');
    }
  }

  // Report to the serial monitor, in case the laptop is hooked up.
  Serial.print("lineDetect[");
  Serial.print(sensorLeft);
  Serial.print("][");
  Serial.print(sensorMiddle);
  Serial.print("][");
  Serial.print(sensorRight);
  Serial.print("]: ");
  Serial.println(mesg);
}

void forward(){
  lcdPrintMotorStatus("forward");
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 
} 

void back(){ 
  lcdPrintMotorStatus("reverse");
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW);
}

void left(){ 
  lcdPrintMotorStatus("left");
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
} 

void right(){
  lcdPrintMotorStatus("right");
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void stop(){
  lcdPrintMotorStatus("stop");
  analogWrite(ENA,0); 
  analogWrite(ENB,0); 
}

/**
 * Returns distance in centimeters.
 */
int ultrasonic_distance_test(){
  // Send a 20 microsecond pulse to the trigger pin, which is the signal to send an ultrasonic beam
  // out the front end.
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(20);
  digitalWrite(ULTRASONIC_TRIGGER, LOW);

  // After we trigger it, the ultrasonic will send us a pulse on the echo pin; the width of the pulse 
  // is the measurement of the echo delay. We divide by the magic-number of 58 to get the distance to 
  // the nearest object, in centimeters.
  float pulseWidth = pulseIn(ULTRASONIC_ECHO, HIGH);
  float distanceCm = pulseWidth / 58;
  Serial.print("Distance_test centimeters=");
  Serial.println(distanceCm);
  return (int) distanceCm;
}

/**
 * Entrypoint to the program; runs once every time the program starts.
 */
void setup() 
{
  // Init the serial line; important for debug messages back to the Arduino Serial Monitor.
  // Make sure you set the baudrate at 9600 in Serial Monitor as well.
  Serial.begin(9600);

  // Init the Arduino pins that we will be using.
  pinMode(SERVO_CONTROL, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(ULTRASONIC_TRIGGER,OUTPUT);
  pinMode(LINETRACKER_LEFT, INPUT);
  pinMode(LINETRACKER_RIGHT, INPUT);
  pinMode(LINETRACKER_MIDDLE, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(LED,OUTPUT);

   // Stop all wheel motors.
  stop();

  // Init the servo by telling it what pin we are connected to.
  ultrasonicServo.attach(SERVO_CONTROL);

  // Turn off the built-in LED.
  digitalWrite(LED,LOW);

  // Initialize the lcd for 20 chars 4 lines and turn on backlight.
  lcd.begin(20,4);         
  lcd.backlight(); 
  
  //-------- Write characters on the display ----------------
  // NOTE: Cursor Position: CHAR, LINE) start at 0  
  lcd.setCursor(1,0);
  lcd.print("Hello, Earthlings!");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print("My name is Robert");
  delay(1000);  
  lcd.setCursor(0,2);
  lcd.print("I COME FOR OCULUS");
  lcd.setCursor(0,3);
}

/**
 * FOLLOW THE LINE ALGORITHM
 * If a sensor is TRUE, then it is probably pointing to the bare floor.
 * If a sensor if FALSE, then it is pointing to black.
 */
void followTheLine() { 
   if (sensorMiddle) {
    if (!sensorRight && !sensorLeft){
        // Nothin' on the right and left. This is the optimal situation.
        // TODO: if we are cruising on the line, perhaps consider speeding up?
        lcdPrintLineDetectStatus("ok");
        forward();
     } else {
        // TODO: what to do when every sensor is reading black? Are we insane? Or perhaps are we rotated
        // 90 degrees and viewing the tape perpendicularly? 
        lcdPrintLineDetectStatus("insanity");
        stop();
     }
  } else if (!sensorRight && sensorLeft) {
    // We've picked up the line on the left sensor, but not the middle or right sensors. That means we should
    // nudge over to the left.
    lcdPrintLineDetectStatus("lft");
    left();
  }
  else if (!sensorLeft && sensorRight) {
    // We've picked up the line on the right sensor, but not the middle or left sensors. That means we should
    // nudge over to the right.
    lcdPrintLineDetectStatus("rgt");
    right();
  }
  else {
    // TODO: No sensors are picking up anything resembling a black line. Crap!
    // What to do? move in random direction?
    lcdPrintLineDetectStatus("lost");
    //forward();
    stop();
  }
}

void goCircle() { 
  lcdPrintLineDetectStatus("forward");
  forward();
  delay(1000);
  lcdPrintLineDetectStatus("left");
  left();
  delay(1000);
  lcdPrintLineDetectStatus("right");
  right();
  delay(1000);
  lcdPrintLineDetectStatus("stop");
  stop();
  delay(1000);
}

/**
 * MAIN LOOP
 */
void loop()
{
  // Point servo forward, and calculate distance to object right in front of us. 
  // (90 degrees is oriented stright ahead, 0 degrees is all the way to the right, and
  // 180 degrees is all the way to the left; this 180 degrees is the full range of 
  // motion of this servo.
  // Note that we could be moving forward right now, or we could be sitting still 
  // and probing a way out of a sticky situation.
  if (!disableCollisionDetection) {
    ultrasonicServo.write(90);
    middleDistance = ultrasonic_distance_test();
    Serial.print("loop begin, center distance=");
    Serial.println(middleDistance);
  }
  
  // Read the line detect sensor.
  lineDetectReadSensor();
  
  // FC: What does this even do?
  // Act based on the distance.
  if (disableCollisionDetection || (!justWentBack && (middleDistance > CRTICAL_DISTANCE))) {
    lcdPrintCollisionDetectStatus("ok");
    followTheLine();
  } else {
    // Inform the monitor that we will be cncaling all line detection stuff in order to avoid the collision.
    lcdPrintLineDetectStatus("paused");

    // FC: This is included in my "what does this even do?" question.
    // If we go there, we are about to collide! Evasive maneuvers!
    justWentBack = false;
    stop();
   
    // Point the sonar to the right, and take a measurement.
    lcdPrintCollisionDetectStatus("analysis");
    ultrasonicServo.write(20);
    delay(1000);
    rightDistance = ultrasonic_distance_test();
    Serial.print("right distance=");
    Serial.println(rightDistance);

    // Point the sensor the the left, and take a measurement.
    ultrasonicServo.write(160);
    delay(1000);
    leftDistance = ultrasonic_distance_test();
    Serial.print("left distance=");
    Serial.println(leftDistance);

    // Reset the servo back to pointing forward.
    ultrasonicServo.write(90);
    
     // FC: What is the "<="? Is it less than or equal to?
     // First: if we are in a corner and no way out, go backward!
    if ((rightDistance <= CRTICAL_DISTANCE) && (leftDistance <= CRTICAL_DISTANCE)) {
       lcdPrintCollisionDetectStatus("retreat");
      digitalWrite(LED,HIGH);
      back();
      delay(1000);
      stop();
       digitalWrite(LED,LOW);
       justWentBack = true;
    } else if (rightDistance > leftDistance) {
      lcdPrintCollisionDetectStatus("right");
      right();
      delay(1000);
      
    } else {
      lcdPrintCollisionDetectStatus("left");
      left();
      delay(1000);
    }
  } 
}


