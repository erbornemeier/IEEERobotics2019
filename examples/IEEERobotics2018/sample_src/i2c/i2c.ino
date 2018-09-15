// Arduino i2c code: https://www.arduino.cc/en/Tutorial/MasterWriter
// I2C functions reference: https://www.arduino.cc/en/Reference/Wire
 
#include <Wire.h>

const int MOTOR_PIN = 3;
const int BLINK_PIN = 13;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x28);
  
  Wire.onReceive(parseReceive);
  Wire.onRequest(parseRequest);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(4, OUTPUT);

  
  Serial.begin(9600);
  while(!Serial) {}
  Serial.println("Hello world");

  while(Wire.available()) {
    char c = Wire.read();
    Serial.println(c);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(BLINK_PIN, HIGH);
  delay(1000);
  digitalWrite(BLINK_PIN, LOW);
  delay(1000);
}

void parseReceive(int numBytes) {
  while(Wire.available()) {
    int c = Wire.read();
    Serial.println(c);
    analogWrite(MOTOR_PIN, c);
  }
}

void parseRequest() {
  while(Wire.available()) {
    int c = Wire.read();
    Serial.println(c);
    analogWrite(MOTOR_PIN, c);
  }
  Wire.write(42);
}

