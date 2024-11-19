#include <wiringpi.h>
#include <>

#define SERVO_PIN 13


void setServoAng(int ang) {
    int duty_cycle = 26 + (ang/180*1024); 
    pwmWrite(SERVO_PIN, duty_cycle); 
    usleep(500000); 
    pwmWrite(SERVO_PIN, 0); 
}

int main() {
    wiringPiSetupGpio(); 
    pinMode(SERVO_PIN, PWM_OUTPUT); 
    pwmSetMode(PWM_MODE_MS); 
    pwmSetRange(1024); 
    pwmSetClock(375); // 50hz 

    for( int i=0; i<10; i++ ) {
        setServoAng(i*18); 
        sleep(10); 
    }

    return 0; 
}