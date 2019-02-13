#include <Servo.h>

#define clawServoPin 12

#define servoMin 10
#define servoLevel 40
#define gripperOpenPower 255
#define gripperClosePower 80
#define gripperOpenTime 800
#define gripperCloseTime 1000

#define MS_PER_DEG 10

enum CLAW_PINS{CLAW_PWM=2, IN1=3, IN2=5};
enum GRIPPER_STATE{GRIPPING, NOT_GRIPPING, UNPOWERED};

class Claw {
  public:

      Servo clawServo;
      GRIPPER_STATE state;
      uint8_t servo_pos;
      
      void Claw_init(){
          clawServo.attach(clawServoPin);
          Servo_SetMin();
          Gripper_Open();
          state = UNPOWERED;
      }

      void Gripper_Open(){
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(CLAW_PWM, gripperOpenPower);
            delay(gripperOpenTime);
            Gripper_UnPower();
      }
      
      void Gripper_Close(){
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(CLAW_PWM, gripperClosePower);  
            delay(gripperCloseTime);
            Gripper_UnPower();
      }
      
      void Gripper_UnPower(){
            analogWrite(CLAW_PWM, 0);
      }
  
      void Servo_SetAngle(uint8_t angle){
          while (servo_pos != angle){
              if      (servo_pos < angle) servo_pos++;
              else if (servo_pos > angle) servo_pos--;
              clawServo.write(servo_pos);
              delay(MS_PER_DEG);
          }
        
      }

      void Servo_SetLevel(){
          Servo_SetAngle(servoLevel);  
      }

      void Servo_SetMin(){
          Servo_SetAngle(servoMin);  
      }
  
  
};
