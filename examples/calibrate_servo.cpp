#include <Arduino.h>
#include <Servo.h>

byte _servo180Pin = 27;
byte _servo0Pin = 26;
byte _servo90Pin = 25;
Servo servo180;
Servo servo0;
Servo servo90;

void setup(){
    Serial.begin(115200);

    servo180.attach(_servo180Pin);
    servo180.write(180);
    
    // servoR.attach(_servoRPin, Servo::CHANNEL_NOT_ATTACHED, 0, 180, 500, 2360);
    servo0.attach(_servo0Pin);
    servo0.write(0);

    servo90.attach(_servo90Pin);
    servo90.write(90);
}

void loop(){

    // for(int i=90; i>=0; i-=10){
    //     servoR.write(i);
    //     Serial.println(i);
    //     delay(1000);
    // }

    // for(int i=90; i<=180; i+=10){
    //     servoR.write(i);
    //     Serial.println(i);
    //     delay(1000);
    // }
}