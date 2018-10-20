#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x28
#define SERVO_PIN 11

Servo servo;

void setup() {
  servo.attach(SERVO_PIN);
  servo.write(5);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  Serial.begin(9600);
}

void loop() {

}

void onReceiveI2C(int numBytes) {
  while(Wire.available()) {
    int value = Wire.read();
    Serial.println("I2C Received: " + value);
    serial.write(value);
  }
}

void onRequestI2C() {
  Serial.println("I2C Requested");
}

int get16BitIntI2C() {
  return (Wire.read() << 8) | Wire.read();
}
