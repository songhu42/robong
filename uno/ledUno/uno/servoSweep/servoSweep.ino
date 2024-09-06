#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
const int pinLed = 13;
const int analogPin = A0; 

int prevVal = 0; 

void setup() {
  pinMode(pinLed,OUTPUT);
  digitalWrite(pinLed,LOW);

  unsigned long hz = 115200; 
  Serial.begin(hz); 

  Serial.print("serial port opened : ");
  Serial.print(hz);

  myservo.attach(8);  // attaches the servo on pin 8 to the Servo object
  
  myservo.write(pos);  
  delay(15); 
}

void loop() {

  if(Serial.available()) {
    String str = Serial.readString(); 
    Serial.println(str);

    String ch = str.substring(0, 1); 

    if (ch == "o") {
      digitalWrite(pinLed,HIGH);
      Serial.println("led1"); 
    } else if(ch == "x") {
      digitalWrite(pinLed,LOW );
      Serial.println("led0"); 
    } else if( ch == "." ) {
      pos += 10; 
      if( pos > 180 ) pos = 180; 
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println("ser" + String(pos)); 
      delay(15); 
    } else if( ch == "," ) {
      pos -= 10; 
      if( pos < 0 ) pos = 0; 
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println("ser" + String(pos)); 

      delay(15); 
    } else {
      unsigned int msgLen = str.length();
      String deg = str.substring(0, msgLen - 2); 
      int degree = deg.toInt(); 

      if( degree < 0 || degree > 180 ) {
        Serial.println("error : degree exceeded.."); 
      } else {
        pos = degree; 
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        Serial.println("ser" + String(pos)); 

        delay(15); 
      }
    }
  }

  int analogVal = analogRead(analogPin); 
  if( prevVal < analogVal-6 || prevVal > analogVal + 6 ) {
    prevVal = analogVal; 
    Serial.println("adc" + String(analogVal)); 
    delay(50); 
  }

  /*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  } 
  */
}