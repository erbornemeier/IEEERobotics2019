#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x28

#define SERVO_PIN 11

Servo servo;
int servoRotation = 5;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(servoRotation);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.begin(9600);
}

void loop() {

}

// numBytes: total number of bytes
// First byte: flag
// Second byte: number of data bytes
void onReceiveI2C(int numBytes) {
  while(Wire.available()) {
    int value = get16BitInt();
    Serial.println(value);
  }
}

void onRequestI2C() {
  Serial.println("I2C Requested");
}

int get16BitInt() {
  return (Wire.read() << 8) | Wire.read();
}
