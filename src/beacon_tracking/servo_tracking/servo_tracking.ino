#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x28

#define SERVO_PIN 11

#define CAM_FRAME_WIDTH 480
#define CAM_FRAME_HEIGHT 600
#define MIN_CAM_X -CAM_FRAME_WIDTH / 2
#define MAX_CAM_X CAM_FRAME_WIDTH / 2
#define SERVO_MIN 0
#define SERVO_MAX 180

Servo servo;
volatile double servoRotation = 90;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(servoRotation);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.begin(9600);
}

void loop() {
    servo.write((int)servoRotation);
    
    delay(10);
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

    if(value > -5 && value < 5) {
        return;
    } else if(value < 0) {
      if(servoRotation < 180) {
        servoRotation -= (double)value / 90;
      }
    } else if(value > 0) {
      if(servoRotation > 0) {
        servoRotation -= (double)value / 90;
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
