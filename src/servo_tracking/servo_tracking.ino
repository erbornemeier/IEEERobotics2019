#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x28

#define SERVO_PIN 11

#define CAM_FRAME_WIDTH 480
#define MIN_CAM_X -CAM_FRAME_WIDTH / 2
#define MAX_CAM_X CAM_FRAME_WIDTH / 2
#define SERVO_MIN 0
#define SERVO_MAX 180

Servo servo;
volatile int servoRotation = 90;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(servoRotation);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.begin(9600);
}

void loop() {
//    Serial.println(servoRotation);
    servo.write(servoRotation);
}

// numBytes: total number of bytes
// First byte: flag
// Second byte: number of data bytes
void onReceiveI2C(int numBytes) {
  
  while(Wire.available()) {
    Wire.read();
    Wire.read();
    
    int value = get16BitInt();
    Serial.println(value);

//    int servoRotation = map(value, MIN_CAM_X, MAX_CAM_X, SERVO_MIN, SERVO_MAX);
//    servo.write(servoRotation);
    if(value < 0) {
      if(servoRotation < 180) {
        servoRotation += 2;
      }
    } else if(value > 0) {
      if(servoRotation > 0) {
        servoRotation -= 2;
      }  
   }
  }
}

void onRequestI2C() {
  Serial.println("I2C Requested");
}

int get16BitInt() {
  return (Wire.read() << 8) | Wire.read();
}
