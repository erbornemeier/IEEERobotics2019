#include <PinChangeInterrupt.h>

//******MOTORS*****************************

//2 wheel bot
enum MOTOR_IDS{L, R, NUM_MOTORS};
enum TYPE {A, B};

//M is mode (direction), E is PWM (speed)
enum MOTOR_PINS{ML=2, EL=3, MR=4, ER=5};

// Encoder info
volatile long enc_counts[2];
unsigned int ENC_PINS[2][2] = { {8, 9},   // Left
                                {10, 11} }; // Right

//*****************************************

void setup() {
  Serial.begin(9600);
  
  for (int i = 0; i < 2; i++){
      pinMode(ENC_PINS[i][A], INPUT_PULLUP);
      pinMode(ENC_PINS[i][B], INPUT_PULLUP);
  }
  pinMode(ML, OUTPUT);
  pinMode(EL, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(ER, OUTPUT);

  // put your setup code here, to run once:
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][A]), LA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][B]), LB_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][A]), RA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][B]), RB_changed,  CHANGE);
}

void loop() {
  Serial.print(enc_counts[0]);
  Serial.print(" , ");
  Serial.println(enc_counts[1]);
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

/*
 * void resetEncoderCounts()
 * Resets each of the encoder counts to zero
 */
void resetEncoderCounts(){
    for (int i = 0; i < 2; i++) enc_counts[i] = 0;
}

/*
 * void encoderCount()
 * Method indirectly used by the interrupts to determine the direction the wheel
 * is going based on which value changed and the state of the one that did
 * not change
 *
 */
void encoderCount(int enc, int type){
    int inc = enc == R ? -1 : 1;

    if (type == A){
      //low to high
      if (digitalRead(ENC_PINS[enc][A]) == HIGH)
          if (digitalRead( ENC_PINS[enc][B]) == LOW)  enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
      //high to low
      else
          if (digitalRead( ENC_PINS[enc][B]) == HIGH) enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
    }

    else
    if (type == B){
      //low to high
      if (digitalRead(ENC_PINS[enc][B]) == HIGH)
          if (digitalRead( ENC_PINS[enc][A]) == HIGH) enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
      //high to low
      else
          if (digitalRead( ENC_PINS[enc][A]) == LOW)  enc_counts[enc]+=inc;
          else                                        enc_counts[enc]-=inc;
    }
}

/*
 * A series of void arg void callback methods that pass arguments needed to
 * the actual interrupt function since interrupts do not allow argument passing
 */
void LA_changed(){encoderCount(L, A);}
void LB_changed(){encoderCount(L, B);}
void RA_changed(){encoderCount(R, A);}
void RB_changed(){encoderCount(R, B);}
