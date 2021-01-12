#define CHA 2
#define CHB 4
#define wheelRadius 0.0335
#define CPR 853
#define PI_val 3.14159 

volatile int count = 0;
volatile double wheelDistance;

void setup() {
  Serial.begin(9600);
  
  pinMode(CHA, INPUT);
  pinMode(CHB, INPUT);

  attachInterrupt(0, interruptionFunction, RISING);

  Serial.println("Encoder test script has initialized ...");
}

void loop() {
  Serial.print("Count: ");
  Serial.print(count);

  wheelDistance = count * 2 * PI_val * wheelRadius / CPR;
  
  Serial.print("\t");
  Serial.print("Wheel distance: ");
  Serial.print(wheelDisance);
  Serial.println(" m");

  delay(22);

}

void interruptionFunction() {
  if (digitalRead(CHA) && !digitalRead(CHB)) {
    count++ ;
  }
  if (digitalRead(CHA) && digitalRead(CHB)) {
    count-- ;
  } 
}
