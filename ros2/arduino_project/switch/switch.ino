#include <Arduino.h>
#include <string.h>

#define PIN_LED 13
#define PIN_SW 2

void setup()
{
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SW, INPUT);

    Serial.begin(115200);
}

bool btn_state = false; 
unsigned long prev_time = 0; 

void loop()
{
    if( millis() - prev_time > 100 ) {
        int btn_pressed = digitalRead(PIN_SW);
        
        if( !btn_state && btn_pressed == HIGH ) {
            btn_state = true; 
            digitalWrite(PIN_LED, HIGH);
            Serial.println("switch_on");
            prev_time = millis(); 
        } else if( btn_state && btn_pressed == LOW ) {
            btn_state = false; 
            digitalWrite(PIN_LED, LOW);
            Serial.println("switch_off");
            prev_time = millis(); 
        }
        
    }  
    
}