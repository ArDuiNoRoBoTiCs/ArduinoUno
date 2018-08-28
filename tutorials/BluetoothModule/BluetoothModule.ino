#define ENA 5 
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13

unsigned char carSpeed = 150;
bool state = LOW;


void forward(){
  digitalWrite(ENA,HIGH); 
  digitalWrite(ENB,HIGH); 
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 
  Serial.println("Forward");
}

void back(){ 
  digitalWrite(ENA,HIGH); 
  digitalWrite(ENB,HIGH); 
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  Serial.println("Back");
}

void left(){ 
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 
  Serial.println("Left");
}  

void right(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Right");
}

void coast(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,HIGH); 
  Serial.println("Coast!");
}

void brake(){
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  Serial.println("Brake!");
}

void stateChange(){
  state = !state;
  digitalWrite(LED, state);
}




void simpleAFunctionMan() {
}

int functionThatReturnsAnInt(int juju) {
  return 666 * juju;
}

void swiggleMilliseconds(int millis) {
  int numIterations = millis / 200;
  for (int n = 0 ; n < numIterations ; n++) {
     swiggle();
  }
}

void swiggle() {
  right();
  delay(100);
  left();
  delay(100);
}

void swiggleSeconds(int secs) {
  swiggleMilliseconds(secs * 1000);
}

//
void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  
  pinMode(LED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);


  simpleAFunctionMan();
  int ret = functionThatReturnsAnInt(2);
  Serial.print("it returns...");
  Serial.println(ret);

  //swiggle();
  //brake();
  forward();
  delay(1500);
  //brake();
  coast();
}

//
void loop() {
  //The Bluetooth serial port to recieve data in the function
  // Serial.println("loop");
  char getstr = Serial.read();
  switch(getstr){
    case 'f': forward(); break;
    case 'b': back(); break;
    case 'l': left(); break;
    case 'r': right(); break;
    case 's': brake(); break;
    case 'a': stateChange(); break;
    case 'p': swiggle(); break;
    default: break;
  }
}
