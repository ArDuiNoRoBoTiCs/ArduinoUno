
#include "unopins.h"
#include "Arduino.h"

void motorInit() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

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

