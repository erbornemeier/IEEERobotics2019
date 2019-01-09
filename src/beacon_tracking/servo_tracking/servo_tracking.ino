#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x28


/*#define CAM_FRAME_WIDTH 480
#define CAM_FRAME_HEIGHT 600
#define MIN_CAM_X -CAM_FRAME_WIDTH / 2
#define MAX_CAM_X CAM_FRAME_WIDTH / 2
*/

#define SERVO_1_PIN 3
#define SERVO_2_PIN 5

#define SERVO_1 1
#define SERVO_2 2

#define SERVO_MIN 0
#define SERVO_MAX 180

#define K_P (1/90.0)

Servo servo1, servo2;
volatile double servo1Rotation = 90, servo2Rotation = 90;

void setup() {
  
  //literally using this for another 5V for the servo
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  servo1.write(servo1Rotation);
  servo2.write(servo2Rotation);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.begin(9600);
  
}

void loop() {
  
    servo1.write((int)servo1Rotation);
    servo2.write((int)servo2Rotation);
    
    delay(10);
}

// numBytes: total number of bytes
// First byte: flag
// Second byte: number of data bytes
void onReceiveI2C(int numBytes) {
  
  while(Wire.available()) {
    
    int cmd = Wire.read();
    int dataSize = Wire.read();
    
    int value = get16BitInt() * K_P;
    
    //Serial.println(value);
    if (cmd == SERVO_1){
        servo1Rotation += value<0?-value:value;
        if (servo1Rotation > SERVO_MAX) servo1Rotation = SERVO_MAX;
        if (servo1Rotation < SERVO_MIN) servo1Rotation = SERVO_MIN;
    }
    else if (cmd == SERVO_2){
        servo2Rotation += value<0?-value:value;
        if (servo2Rotation > SERVO_MAX) servo2Rotation = SERVO_MAX;
        if (servo2Rotation < SERVO_MIN) servo2Rotation = SERVO_MIN;
    }
    
  }
}

void onRequestI2C() {
  Serial.println("I2C Requested");
}

int get16BitInt() {
  return (Wire.read() << 8) | Wire.read();
}
