#include "PCA9685.h"
PCA9685 pwmController(Wire);

PCA9685_ServoEval pwmServo0(152,328,494);
PCA9685_ServoEval pwmServo1(338,480,550);
PCA9685_ServoEval pwmServo2(169,304,435);
PCA9685_ServoEval pwmServo3(220,380,520); 
PCA9685_ServoEval pwmServo4(220,220,400); 
int arr[4];

void setup() {
    Serial.begin(9600);               // Begin Serial and Wire1 interfaces
    Wire.begin();
    Serial.setTimeout(1);

    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line
    pwmController.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer

    pwmController.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length

    pwmController.setChannelPWM(0, pwmServo0.pwmForAngle(0));
    pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(-45));
    pwmController.setChannelPWM(2, pwmServo2.pwmForAngle(-45));
    pwmController.setChannelPWM(3, pwmServo3.pwmForAngle(-45));
    pwmController.setChannelPWM(4, pwmServo4.pwmForAngle(0));
    
}
void loop() {
  
 while (Serial.available()>=5){
  for (int i = 0; i < 5; i++){
      arr[i] = Serial.read();
    }


    pwmController.setChannelPWM(4, pwmServo4.pwmForAngle(arr[4]-90));
    pwmController.setChannelPWM(0, pwmServo0.pwmForAngle(arr[0]-90));
    pwmController.setChannelPWM(1, pwmServo1.pwmForAngle(arr[1]-135));
    pwmController.setChannelPWM(2, pwmServo2.pwmForAngle(arr[2]-135));
    pwmController.setChannelPWM(3, pwmServo3.pwmForAngle(arr[3]-135));
    


 }
}
