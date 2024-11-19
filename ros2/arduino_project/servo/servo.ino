#include <Arduino.h>
#include <string.h>

#include <Servo.h> 

Servo myservo; 

void setup()
{
    myservo.attach(6); 
    Serial.begin(115200);
}
void moveServo(int to) {
    int from = myservo.read(); 
    int step = (to - from); 
    int sign = 1; 
    if(step < 0) {
        step = step * -1; 
        sign = -1; 
    }

    int pos = from; 
    for(int i=0; i<step; i++ )
    pos += step; 
    myservo.write(pos);                  // sets the servo position according to the scaled value
    delay(100);  
}
void loop()
{
    static String buffer;
    if (Serial.available() > 0)
    {
        buffer = Serial.readStringUntil('\n');
        if (buffer.substring(0,4) == "move") {
            int pos = buffer.substring(4, 6).toInt(); 
            if( pos < 0 ) pos = 0; 
            if( pos > 180 ) pos = 180; 

            moveServo(pos); 
        }
        Serial.print("Echo : ");
        Serial.println(buffer);
    }
    delay(100); 
}