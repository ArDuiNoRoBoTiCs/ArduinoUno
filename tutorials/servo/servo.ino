#include <Servo.h>
Servo myservo;

int Echo = A4;
int Trig = A5;

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13

int justWentBack = false;
int carSpeed  = 100;
int rightDistance = 0, leftDistance = 0, middleDistance = 0;


void forward(){
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 
  Serial.println("Forward");
}

void back(){ 
  analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW);
  Serial.println("Back");
}

void left(){ 
 analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
  Serial.println("Left");
} 

void right(){
 analogWrite(ENA,carSpeed); 
  analogWrite(ENB,carSpeed); 
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Right");
}

void stop(){
  analogWrite(ENA,0); 
  analogWrite(ENB,0); 
  //Serial.println("Stop!");
}


/**
 * Returns distance in centimeters.
 */
int Distance_test(){
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  
  float pulseWidth = pulseIn(Echo, HIGH);
  //Serial.print("Distance_test pulseWidth=");
  //Serial.println(pulseWidth);
  float distanceCm = pulseWidth / 58;
  //Serial.print("Distance_test centimeters=");
  //Serial.println(distanceCm);
  return (int) distanceCm;
  
}

void setup() {
  Serial.begin(9600);

  // Init the pins.
  pinMode(3, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(Trig,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(LED,OUTPUT);

  // stop all wheel motors.
  stop();

   // Init the servo.
   myservo.attach(3);

   digitalWrite(LED,LOW);

   // init state machine for driving logic.
   justWentBack = false;
}


#define CRTICAL_DISTANCE 20

void loop() {
  // Point servo forward.
  myservo.write(90);

  // Calculate distance to object right in front.
  middleDistance = Distance_test();
  Serial.print("loop begin, center distance=");
  Serial.println(middleDistance);

  // Act based on the distance.
  if (!justWentBack && (middleDistance > CRTICAL_DISTANCE)) {
    // The distance in front of the robot is > 20, so go forward.
    forward();
  } else {
    // If we go there, we are about to collide! Evasive maneuvers!
    justWentBack = false;
    stop();
   
    // Ppint the sonar to the right, and take a measurement.
    myservo.write(20);
    delay(1000);
    rightDistance = Distance_test();
    Serial.print("right distance=");
    Serial.println(rightDistance);

    // Point the sensor the the left, and take a measurement.
    myservo.write(160);
    delay(1000);
    leftDistance = Distance_test();
    Serial.print("left distance=");
    Serial.println(leftDistance);

    // Reset the sero back to pointing forward.
    myservo.write(90);

     // First: if we are in a corner and no way out, go backward!
    if ((rightDistance <= CRTICAL_DISTANCE) && (leftDistance <= CRTICAL_DISTANCE)) {
      digitalWrite(LED,HIGH);
      back();
      delay(1000);
      stop();
       digitalWrite(LED,LOW);
       justWentBack = true;
    } else if (rightDistance > leftDistance) {
      right();
      delay(1000);
      
    } else {
      left();
      delay(1000);
    }
  } 
}

  














































