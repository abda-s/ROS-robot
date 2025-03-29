#define outputA 17
#define outputB 16

volatile int counter = 0; 
volatile int aLastState;  

void IRAM_ATTR handleEncoder() {
  int aState = digitalRead(outputA);
  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {
      counter++;
    } else {
      counter--;
    }
  }
  aLastState = aState;
}


void setup() {
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  Serial.begin(9600);
  aLastState = digitalRead(outputA);
  attachInterrupt(digitalPinToInterrupt(outputA), handleEncoder, CHANGE);
}

void loop() {
  static int lastCounter = 0; 
  if (lastCounter != counter) {
    Serial.print("Position: ");
    Serial.println(counter);
    lastCounter = counter;
  }
}

