
//******MOTORS*****************************

//2 wheel bot
enum MOTOR_IDS{L, R, NUM_MOTORS};

//M is mode (direction), E is PWM (speed)
enum MOTOR_PINS{ML=2, EL=3, MR=4, ER=5};

//*****************************************

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  delay(5000);
  setMotor(L, -150);
  setMotor(R, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  setMotor(L, 150);
  setMotor(R, 150);
  delay(2000);
  stopMotors();
  delay(1000);
  setMotor(L, -150);
  setMotor(R, 150);
  delay(2000);
  stopMotors();
  

}

//set motor speeds
//example -> setMotor(R, -150); for right motor to move in reverse
void setMotor(int m, int pwm){

    //get motor pins
    int M = (m==L?ML:MR);
    int E = (m==L?EL:ER);
                                     
    //set direction and speed                                                           
    digitalWrite(M, pwm > 0 ? HIGH:LOW);                                                              
    analogWrite (E, abs(pwm));
                                                  
}

void stopMotors(){
    setMotor(L,0);
    setMotor(R,0);  
}
