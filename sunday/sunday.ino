
#include "unopins.h"
#include "unomotor.h"

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
  Serial.begin(9600);
  Serial.println("setup");

 // Init the LED.
  pinMode(LED, OUTPUT);

// Init the motor library.
  motorInit();
}



void loop() {
 int val = analogRead(0);
 Serial.print("loop: ");
 Serial.println(val);

 // Blink the LED.
  /*digitalWrite(LED, HIGH);
  delay(1500);
  digitalWrite(LED, LOW);
  delay(1000); (*/

  // Control the Motor.
  //high();
  //forwardRight();
}
