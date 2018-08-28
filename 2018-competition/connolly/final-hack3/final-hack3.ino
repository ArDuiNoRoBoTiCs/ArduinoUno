#include <Wire.h>
#include <Servo.h>
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <LiquidCrystal_I2C.h>

/**
 * Distance a wall needs to be (in centimeters) to pose a threat.
 */
#define CRTICAL_DISTANCE 20

#define LOST_TIME 300 // in milliseconds
#define REVERSE_TIME (LOST_TIME * 4)

/**
 * Starting car speed. Note that 90 and under results in a loud whining sound from the 
 * motors, and the wheels dont turn - not enough power.
 */
#define CAR_SPEED_ATTACH 100
#define CAR_SPEED_HIGH  100 //200//120: good, but still leaped off curve every other time.
#define CAR_SPEED_HARD_TURN 180 //200 
#define CAR_SPEED_BACK  100 //120 // line detection:200
#define CAR_SPEED_TURN 180 //160 //180 // line detection: 180
#define CAR_SPEED_OBSTACLE 100

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
boolean disableCollisionDetection = false; // HACK
Servo ultrasonicServo;
int rightDistance = 0;
int leftDistance = 0;
int centerDistance = 0;
int justWentBack = false;

// *** Line Detect subsystem
int sensorRight = 0;
int sensorMiddle = 0;
int sensorLeft = 0;
bool lost = false;
unsigned long lostTime = 0;
unsigned long hopelesslyLostTime = 0;
bool hasEverMoved = false;

// *** Motor subsystem


int justDie = false;

/*************** LCD (Liquid Crystal Display) SUBSYSTEM ****************/


/**
 * Takes over the fourth (last) row of the LCD.
 */
void lcdPrintMotorStatus(String mesg, int velocity) {
  static const String label = "mtr: ";
  static const int rowNumber = 3;
  
  lcd.setCursor(0,rowNumber);
  lcd.print(label);
  lcd.setCursor(label.length(), rowNumber);

  String fmtMessage = mesg + " (" + String(velocity) + ")";
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
  static const String label = "col: ";
  static const int rowNumber = 2;
  static unsigned long lastWriteTime = 0;
  static int lastWriteDistance = 0;

  // Only update the distance every half-second, or it looks too freaky.
  unsigned long now = millis();
  if (now - lastWriteTime > /*125 */ 0) {
    lastWriteDistance = centerDistance;
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
  if (now - lastWriteTime > /*500 */ 0) {
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
}


/*************** LINE DETECT SUBSYSTEM ****************/

/**
 * Reads the current state of the line tracker sensors, and records
 * it in the sensorRight, sensorMiddle, and sensorLeft global variables.
 */

boolean lineDetectReadRight() {
  sensorRight = !digitalRead(LINETRACKER_RIGHT);
  return sensorRight;
}

boolean lineDetectReadMiddle() {
  sensorMiddle = !digitalRead(LINETRACKER_MIDDLE);
  return sensorMiddle;
}

boolean lineDetectReadLeft() {
  sensorLeft = !digitalRead(LINETRACKER_LEFT);
  return sensorLeft;
}

void lineDetectReadSensors() {
  lineDetectReadRight();
  lineDetectReadMiddle();
  lineDetectReadLeft();
}

boolean lineDetectReadAny() {
  lineDetectReadSensors();
  if (sensorRight || sensorMiddle || sensorLeft) {
    digitalWrite(LED,HIGH);
    return true;
  }

  digitalWrite(LED,LOW);
  return false;
}
 



/*************** MOTOR SUBSYSTEM ****************/


void forward(int velocity) {
  lcdPrintMotorStatus("forward", velocity);
  analogWrite(ENA, velocity); 
  analogWrite(ENB, velocity); 
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
} 

void back(int velocity) { 
  lcdPrintMotorStatus("reverse", velocity);
  analogWrite(ENA, velocity); 
  analogWrite(ENB, velocity); 
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
}

void left(int velocity) { 
  lcdPrintMotorStatus("left", velocity);
  analogWrite(ENA, velocity / 2); 
  analogWrite(ENB, velocity); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
} 

void leftHard(int velocity) { 
  lcdPrintMotorStatus("left", velocity);
  analogWrite(ENA, velocity);
  analogWrite(ENB, velocity); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
} 

void right(int velocity) {
  lcdPrintMotorStatus("right", velocity);
  analogWrite(ENA, velocity); 
  analogWrite(ENB, velocity / 2); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rightHard(int velocity)  {
  lcdPrintMotorStatus("right", velocity);
  analogWrite(ENA, velocity); 
  analogWrite(ENB, velocity);  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stop() {
  lcdPrintMotorStatus("stop", 0);
  analogWrite(ENA,0); 
  analogWrite(ENB,0); 
}


/*************** COLLISION DETECTION SUBSYSTEM ****************/


/**
 * Returns distance in centimeters.
 */
float ultrasonic_distance_test_internal(){
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
  return distanceCm;
}

int ultrasonic_distance_test() {
  const int numTests = 3;
  float results[numTests];
  float sum = 0;
  for (int i = 0 ; i < numTests ; i++) {
    results[i] = ultrasonic_distance_test_internal();
    sum += results[i];
  }

  float avg = sum / numTests;
  Serial.print("Distance_test avg centimeters=");
  Serial.println(avg);
  return (int) avg; 
  //return ultrasonic_distance_test_internal();
}


/**
 * FOLLOW THE LINE ALGORITHM
 * If a sensor is TRUE, then it is probably pointing to the bare floor.
 * If a sensor if FALSE, then it is pointing to black.
 */
void followTheLine() { 

  // Default state to be 
  bool oldLost = lost;
  bool newLost = false;

 if (lineDetectReadMiddle()) {
    lcdPrintLineDetectStatus("ok");
    forward(CAR_SPEED_HIGH);
    hasEverMoved = true; 
  } else if (lineDetectReadLeft()) { // only left
    // We've picked up the line on the left sensor, but not the middle or right sensors. That means we should
    // nudge over to the left.
    lcdPrintLineDetectStatus("lft-hard");
     leftHard(CAR_SPEED_TURN); // line detection:leftHard
      while(lineDetectReadLeft());
  }
  else if (lineDetectReadRight()) { // only right
    // We've picked up the line on the right sensor, but not the middle or left sensors. That means we should
    // nudge over to the right.
    lcdPrintLineDetectStatus("rgt-hard");
     rightHard(CAR_SPEED_TURN); // linedtection:rightHard
     while(lineDetectReadRight());
  } else { 
    // All sensors are off!
    // No sensors are picking up anything resembling a black line. Crap! What to do? 
    // We have probably hit a tight curve and ran right off the line. Let's back the truck up.
    // Manage the lost state machine.
     newLost = true;
     unsigned long now = millis();
    if (!oldLost) {
      // Newly transitioned to the lost state.
      lcdPrintLineDetectStatus("lst1");
      lostTime = millis();
      hopelesslyLostTime = 0;
    } else {
      if ((now - lostTime) < LOST_TIME) {
        // Do nothing; keep coasting. Perhaps we will bump back.
        lcdPrintLineDetectStatus("lst2");
      } else {
        // hopelessly lost.
        if (hopelesslyLostTime == 0) {
          lcdPrintLineDetectStatus("lst3");
          hopelesslyLostTime = millis();
          stop();
        } else if (hasEverMoved && ((now - hopelesslyLostTime) < REVERSE_TIME)) {
          // We've been lost too long.
          lcdPrintLineDetectStatus("lst4");
          back(CAR_SPEED_BACK);
          boolean foundIt = delayWithLineDetect(REVERSE_TIME);
          if (foundIt) {
            lostTime = 0;
            hopelesslyLostTime = 0;
          }
          //stop();
        } else {
          // eternally lost. abandonded.
          lcdPrintLineDetectStatus("lst5");
          //digitalWrite(LED,HIGH);
          stop();
        } 
      }
    }
  }

  // Record new state.
  lost = newLost;
}

/*************** ENTRYPOINT ****************/

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
  lcd.print("666[{|ROBERT|}]666");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print("My name is Robert");
  delay(1000);  
  lcd.setCursor(0,2);
  lcd.print("I COME FOR OCULUS");
  lcd.setCursor(0,3);

  // Stop all wheel motors.
  stop();

  // State machine for ffoolow-the-line.
  lost = false;
  lostTime = 0;
  hasEverMoved = false;
}

boolean delayWithLineDetect(int millisDelay) {
  boolean foundLine = false;
  int start = millis();
  while (!foundLine && ((millis() - start) < millisDelay)) {
      foundLine |= lineDetectReadAny();
      //delay(1);
  }
  return foundLine;
}

/**
 * MAIN LOOP
 */
void loop()
{
   if (justDie)
   {
    lcdPrintLineDetectStatus("lock");
    return;
   } 

  // Point servo forward, and calculate distance to object right in front of us. 
  // (90 degrees is oriented stright ahead, 0 degrees is all the way to the right, and
  // 180 degrees is all the way to the left; this 180 degrees is the full range of 
  // motion of this servo.
  // Note that we could be moving forward right now, or we could be sitting still 
  // and probing a way out of a sticky situation.
  if (!disableCollisionDetection) {
    ultrasonicServo.write(90);
    centerDistance = ultrasonic_distance_test();
    Serial.println("loop begin, center distance=" + String(centerDistance));
  }
  
  // Let's take action, based on what our sensors are currently telling us. We'll take into consideration
  // the distance to a possible collision in front of us, and the presence of the black line underneath us.
  if (disableCollisionDetection) {
    lcdPrintCollisionDetectStatus("disabled");
    followTheLine();
  } else if (centerDistance > CRTICAL_DISTANCE) {
    lcdPrintCollisionDetectStatus("ok");
    followTheLine();
  } else {
    // Inform the monitor that we will be canceling all line detection stuff in order to avoid the collision.
    lcdPrintLineDetectStatus("alert");
    stop();

    // Go back; if we don't we will hit the corner of the box when we are passing it at an angle. We tried an alternate approach of 
    // reducing the critical_distance, but that eneded up being too flakey (too many bad distance measurements).
    back(CAR_SPEED_BACK);
    delay(250);
   
     // Keep rotating right until the coast is clear.
    int numBoosts = 0;
    int frontDetect = ultrasonic_distance_test();
    do {
      lcdPrintCollisionDetectStatus("turn (" + String(frontDetect) + ")");
      numBoosts++;
      rightHard(CAR_SPEED_HARD_TURN);
      delay(500); // TODO: look at this magic number
      frontDetect = ultrasonic_distance_test();
    } while (frontDetect <= CRTICAL_DISTANCE);
    stop();

    // Turn the servo all the way to the left, to measure distance to wall.
    ultrasonicServo.write(180);
    delay(1000); // TODO: tighten that up; we dont need a second! Only moving 70 degrees

    // Main turning loop; go forward until we cant see the box to the left, then turn left until we see it again.
    boolean lookForLine = false;
    boolean foundLine = false;
    int sideDetect = 0;
    boolean continueLoop = true;
    while (continueLoop) {
      lcdPrintLineDetectStatus("search");
      
      // detect our left side. Keep going straight until the distance gets greater.
      sideDetect = ultrasonic_distance_test();

      // Move forward, until we have a clear view to the left.
      int numForwardThrusts = 0;
      do {
        lcdPrintCollisionDetectStatus("sneak (" + String(sideDetect) + ")");
        if (lookForLine) 
          foundLine = lineDetectReadAny();
        forward(CAR_SPEED_OBSTACLE);
        numForwardThrusts++;
        
        sideDetect = ultrasonic_distance_test();
        if (lookForLine) 
          foundLine |= lineDetectReadAny();
        continueLoop = lookForLine ? !foundLine : true;
      } while (continueLoop && ((sideDetect <= CRTICAL_DISTANCE * 2) || (numForwardThrusts < 5)));

      Serial.println("broke out march due to " + String(continueLoop) + ", " + String(sideDetect) + ", " + String(numForwardThrusts));
      stop();
      //justDie = true;
      //return;
      
     if (continueLoop) {
        // continue moving forward just a smidge.
        foundLine = delayWithLineDetect(500);
        if (foundLine) {
          continueLoop = false;
          break;
        }
    
        // Turn left until we detect the box again.
        sideDetect = ultrasonic_distance_test();
        int startTurn = millis();
        do {
          lcdPrintCollisionDetectStatus("boost (" + String(frontDetect) + ")");
          foundLine = lineDetectReadAny();
          leftHard(CAR_SPEED_HARD_TURN);
          numBoosts++;
          
          sideDetect = ultrasonic_distance_test();
          foundLine |= lineDetectReadAny();
          continueLoop = lookForLine ? !foundLine : true;
        } while (continueLoop && (sideDetect > CRTICAL_DISTANCE) && (millis() - startTurn < 3000));

        // correct back to the right, since we went slightly too left.
        if (continueLoop) {
          rightHard(CAR_SPEED_HARD_TURN);
          foundLine = delayWithLineDetect(200);
          if (foundLine)
            continueLoop = false;
          stop();
        }
     } 

      // After going thru this loop once, start looking for the line.
      lookForLine = true;
    }

   // We should be right on the line now. turn onto it.
   foundLine = false;
   int nudges = 0;
   do {
    nudges++;
    foundLine = lineDetectReadMiddle();
    rightHard(CAR_SPEED_HARD_TURN);
    foundLine |= delayWithLineDetect(100);
    forward(CAR_SPEED_ATTACH);
    foundLine |= delayWithLineDetect(150);
    lcdPrintLineDetectStatus("attach");
   } while((nudges < 10) || !foundLine);

   if (foundLine) {
     nudges = 0;
     do {
      nudges++;
      foundLine = lineDetectReadMiddle();
      forward(CAR_SPEED_ATTACH);
      foundLine |= delayWithLineDetect(50);
      leftHard(CAR_SPEED_HARD_TURN);
      foundLine |= delayWithLineDetect(100);
      forward(CAR_SPEED_ATTACH);
      foundLine |= delayWithLineDetect(50);
      lcdPrintLineDetectStatus("attach");
     } while((nudges < 3) || !foundLine);
   }

    lcdPrintCollisionDetectStatus("done: " + String(sideDetect));
    
    /*// Point the sonar to the right, and take a measurement.
    lcdPrintCollisionDetectStatus("analysis");
   ultrasonicServo.write(20);
    rightDistance = ultrasonic_distance_test();
    Serial.print("right distance=");
    Serial.println(rightDistance);

    // Point the sensor the the left, and take a measurement.
    ultrasonicServo.write(160);
    delay(1000); // TODO: tighten that up; we dont need a second! However, moving full 140 degrees, might take longer than above.
    leftDistance = ultrasonic_distance_test();
    Serial.print("left distance=");
    Serial.println(leftDistance); */

    // Reset the servo back to pointing forward. Dont bother waiting for it to finish.
    //ultrasonicServo.write(90);  
    

    //leftHard(CAR_SPEED_HARD_TURN);
    //delay(1200);

    //forward(CAR_SPEED_HIGH);
    //delay(1000);

     //stop();
    //justDie = true;
  }
    
    /*left(CAR_SPEED_LOW);
    delay(500);
    forward(CAR_SPEED_MED);
    delay(300);
    left(CAR_SPEED_LOW);
    delay(500);
    forward(CAR_SPEED_MED);
    delay(500); 
    
    
      rightHard(CAR_SPEED_HARD_TURN);
    delay(1200);
    
    //forward(CAR_SPEED_HIGH);
    //delay(1000);

    leftHard(CAR_SPEED_HARD_TURN);
    delay(1200);

    //forward(CAR_SPEED_HIGH);
    //delay(1000);*/
    
    // If we go there, we are about to collide! Evasive maneuvers!
    /*justWentBack = false;
    stop();*/
}




 //   if (justDie)
//   {
//    lcdPrintLineDetectStatus("lock");
//    return;
//   } 
//
//  boolean lookForLine = true;
//  boolean continueLoop = true;
//  int numForwardThrusts = 0;
//  int sideDetect = 0;
//      do {
//        //lcdPrintCollisionDetectStatus("sneak (" + String(sideDetect) + ")");
//        forward(90);
//        //delay(10);
//        numForwardThrusts++;
//        //sideDetect = ultrasonic_distance_test();
//        boolean foundLine = lineDetectReadAny();
//        lcdPrintLineDetectStatus("srch " + String(foundLine));
//        continueLoop = lookForLine ? !foundLine : true;
//      } while (continueLoop/* && ((sideDetect <= CRTICAL_DISTANCE * 2) || (numForwardThrusts < 2))*/);
//
//      stop();
//      justDie = true;
//      return;

