const int LED13 = 13; 
const int LED10 = 10;
const int PIN_IN2 = 2;
const int PIN_INA0 = A0; // 14 

void setup() {
  Serial.begin(115200); 
  Serial.println("Setup Completed.. Hello. Arduino uno~~"); 
  Serial.println(PIN_INA0); 

  pinMode(LED10, OUTPUT); 
  pinMode(LED13, OUTPUT); 
  pinMode(PIN_IN2, INPUT); 

  digitalWrite(LED13, HIGH); 

  delay(1000); 

  digitalWrite(LED13, LOW); 

}

int curVal = 0; 

void loop() {
  // toggle by push button : digitalRead
  int in = digitalRead(PIN_IN2); 
  if( in == 1 ) {
    if( curVal == 0 ) digitalWrite(LED13, HIGH);
    else digitalWrite(LED13, LOW);
    curVal = (curVal+1)%2; 

    delay(500); 
  } 

  // 입력 읽기 RX
  if(Serial.available()) {
    char ch = Serial.read(); 
    Serial.print(ch);

    switch(ch) {
      case 'x' : 
        digitalWrite(LED13, LOW); 
        digitalWrite(LED10, LOW); 
        break; 
      case 'o' : 
        digitalWrite(LED13, HIGH); 
        digitalWrite(LED10, HIGH); 
        break; 
      case 'b' : 
        brightLED(); 
        break; 
      default:
        break; 
    }
  }

}

void brightLED() {
    // led 밝기 조절 .. 

  for(int i=1; i<10; i++) {
    int cnt = 0; 
    setLEDAnalog(i); 

    // while(true) {
    //   setLED(i); 
    //   cnt++;

    //   if(cnt > 100) break; 
    // }
  }
}

void setLED(int bright) {
  digitalWrite(LED13, HIGH); 
  delay(bright); 

  digitalWrite(LED13, LOW); 
  delay(10-bright); 
}

void setLEDAnalog(int bright) {
  analogWrite(LED10, bright*25); 
}
