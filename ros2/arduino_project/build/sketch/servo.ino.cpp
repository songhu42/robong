#line 1 "/home/song/apps/robong/ros2/arduino_project/servo/servo.ino"
#include <Arduino.h>
#include <string.h>

#include <Servo.h> 

Servo myservo; 

#line 8 "/home/song/apps/robong/ros2/arduino_project/servo/servo.ino"
void setup();
#line 14 "/home/song/apps/robong/ros2/arduino_project/servo/servo.ino"
void loop();
#line 8 "/home/song/apps/robong/ros2/arduino_project/servo/servo.ino"
void setup()
{
    myservo.attach(6); 
    Serial.begin(115200);
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

            myservo.write(pos);                  // sets the servo position according to the scaled value
            delay(100);         
        }
        Serial.print("Echo : ");
        Serial.println(buffer);
    }
    delay(100); 
}
