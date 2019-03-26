//#include <Servo.h>
#include <Servo_Hardware_PWM.h>
#define clawServoPin 45
#define DEG_TO_US(X) (X*(2000/180.0) + 500)

#define servoMin 10
#define servoLevel 40
#define gripperOpenPower 255
#define gripperClosePower 80
#define gripperOpenTime 800
#define gripperCloseTime 1500

#define MS_PER_DEG 2

enum CLAW_PINS{CLAW_PWM=2, IN1=3, IN2=5};
enum GRIPPER_STATE{GRIPPING, NOT_GRIPPING};

class Claw {
  public:

      Servo clawServo;
      GRIPPER_STATE state;
      uint8_t servo_pos;
      bool gripper_closed = true;
      
      void Claw_init(){
          clawServo.attach(clawServoPin);
          Servo_SetMin();
          Gripper_Open();
      }

      void Gripper_Open(){
            if (gripper_closed){
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                analogWrite(CLAW_PWM, gripperOpenPower);
                delay(gripperOpenTime);
                Gripper_UnPower();
                gripper_closed = false;
            }
            
      }
      
      void Gripper_Close(){
            if (!gripper_closed){
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                analogWrite(CLAW_PWM, gripperClosePower);  
                delay(gripperCloseTime);
                Gripper_UnPower();
                gripper_closed = true;
            }
            
      }
      
      void Gripper_UnPower(){
            analogWrite(CLAW_PWM, 0);
      }
  
      void Servo_SetAngle(uint8_t angle){
        clawServo.writeMicroseconds(DEG_TO_US(angle));
        return;
//          while (servo_pos != angle){
//              if      (servo_pos < angle) servo_pos++;
//              else if (servo_pos > angle) servo_pos--;
//              delayWithWrite();
//          }
        
      }

      void Servo_SetLevel(){
          Servo_SetAngle(servoLevel);  
      }

      void Servo_SetMin(){
          Servo_SetAngle(servoMin);  
      }

      void delayWithWrite(){
          long s = millis();
          while (millis()-s < MS_PER_DEG) clawServo.writeMicroseconds(servo_pos);
      }
  
  
};
