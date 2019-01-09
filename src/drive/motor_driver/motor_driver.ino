#include <PinChangeInterrupt.h>

/*****************************************
* MOTORS AND ENCODERS
*****************************************/

//helper enums
enum MOTOR_IDS{L, R, NUM_MOTORS};
enum ENC_TYPE {A, B};

//M is mode (direction), E is PWM (speed)
enum MOTOR_PINS{ML=8, EL=10, MR=7, ER=9, ENABLE=4};

// Encoder info
volatile long encCounts[2];
unsigned int ENC_PINS[NUM_MOTORS][2] = { { 2,  3},   // Left
                                         { 5,  6} }; // Right

/*****************************************
* POSITION CONTROL & MOTOR FEEDBACK
*****************************************/

//control values
#define MAX_SPEED         70
#define MIN_SPEED         55
#define MAX_CORRECTION    30
#define K_P               0.008
#define K_I               0.0
#define K_DIFF            0.15
#define ZERO_ERROR_MARGIN  50

//robot specs
#define COUNTS_PER_REV       3200
#define WHEEL_DIAMETER       3.45
#define DISTANCE_PER_REV    (WHEEL_DIAMETER*PI)  
#define ROBOT_WIDTH          9.85
#define TURN_CIRCUMFERENCE  (ROBOT_WIDTH*PI)
#define ANGLE_PER_REV       ((DISTANCE_PER_REV/TURN_CIRCUMFERENCE) * 360)


volatile float actual_position[2] = {0.0,   // x_actual
                                     0.0 }; // y_actual

volatile float target_position[2] = {0.0,   // x_target
                                     0.0 }; // y_target
                                     
/*****************************************/

/*
 * void setup()
 */
void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < NUM_MOTORS; i++){
      pinMode(ENC_PINS[i][A], INPUT_PULLUP);
      pinMode(ENC_PINS[i][B], INPUT_PULLUP);
  }
  pinMode(ENABLE,  OUTPUT);
  digitalWrite(ENABLE, HIGH);
  pinMode(ML, OUTPUT);
  pinMode(EL, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(ER, OUTPUT);

  // attach interrupts to four encoder pins
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][A]), LA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][B]), LB_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][A]), RA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][B]), RB_changed,  CHANGE);
}

/*
 * void loop()
 */
void loop() {

    delay(1000);
    drive(36);
    turn(180);
    drive(36);
    turn(180);
    
    while(true); //stop here****
}

/*                                                                              
 * void drive()                                                                 
 * Takes in a distance in inches and drives forwards/backwards until value is reached
 */                                                                             
void drive(float distance){ 

    long encCountGoals[NUM_MOTORS];
    long encErrors[NUM_MOTORS] = {0, 0};
    long lastErrors[NUM_MOTORS];
                                                    
    resetEncoderCounts();                                                       

                                                                
    for (int i = 0; i < NUM_MOTORS; i++) {                                              
        encCountGoals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV);
    }                                                                        

    long lastDiff = millis();
    do{                                                                            
        do {                                                                        
            for (int i = 0; i < NUM_MOTORS; i++){                                            
                lastErrors[i] = encErrors[i];  
                encErrors[i] = encCountGoals[i] - encCounts[i];                                                              
                if (lastErrors[i] != encErrors[i]) lastDiff = millis();
                                                                                   
                int motorVal = errorToMotorOut(encErrors, i, false);                                                                 
                setMotor(i, motorVal);                                             
            }    
            if (millis() - lastDiff > 2000){
                stopMotors();
                return;  
            }
            
                                                                                                                                        
        } while (!isZero(encErrors));
        
        stopMotors();
        //recalculate errors after some delay without overshoot offset
        delay(500);
        for (int i = 0; i < NUM_MOTORS; i++){
                encErrors[i] = encCountGoals[i] - encCounts[i];  
        }

    }while (!isZero(encErrors));
                                           
    stopMotors();                                                                            
                                                                   
}

/*                                                                              
 * void turn()                                                                 
 * Takes in an angle in degrees and rotates until that value is reached
 */                                                                             
void turn(float angle){ 

    long encCountGoals[NUM_MOTORS];
    long encErrors[NUM_MOTORS] = {0, 0};
    long lastErrors[NUM_MOTORS];
                                                    
    resetEncoderCounts();                                                       
                                                                
    for (int i = 0; i < NUM_MOTORS; i++){                                             
        encCountGoals[i] = (long) (angle  * COUNTS_PER_REV / ANGLE_PER_REV); 
        if (i == R) encCountGoals[i] *= -1;  
    }                                                                       

    long lastDiff = millis();

    do{                                                                            
        do {  
            bool same = true;                                                                      
            for (int i = 0; i < NUM_MOTORS; i++){
                lastErrors[i] = encErrors[i];                                            
                encErrors[i] = encCountGoals[i] - encCounts[i];                                                              
                if (lastErrors[i] != encErrors[i]) lastDiff = millis();
                                                                             
                int motorVal = errorToMotorOut(encErrors, i, true); 
                                                                                  
                setMotor(i, motorVal);                                             
            }    
            if (millis() - lastDiff > 2000){
                stopMotors();
                return; 
            }
           
                                                                                                                                        
        } while (!isZero(encErrors));
        
        stopMotors();
        //recalculate errors after some delay without overshoot offset
        delay(500);
        for (int i = 0; i < NUM_MOTORS; i++){
                encErrors[i] = encCountGoals[i] - encCounts[i];  
        }

    }while (!isZero(encErrors));
                                           
    stopMotors();                                                                            
                                                                   
}       


void printErrors(long* errors){
  
    for(int i = 0; i < NUM_MOTORS; i++){
        Serial.print(errors[i]);
        if (i != NUM_MOTORS-1)
            Serial.print(", ");
    }
    Serial.println("");

}
/*                                                                              
 * int errorToMotorOut()                                                        
 * Takes in an error and gain value and converts it to a motor output with an   
 * upper bound check of max speed                                               
 */  
int errorToMotorOut(long* errors, int motor, bool turning){
    //base value from P controller
    int baseVal = (int) (K_P * errors[motor]);
    if (baseVal > MAX_SPEED)  baseVal =  MAX_SPEED;
    if (baseVal < -MAX_SPEED) baseVal = -MAX_SPEED;
    
    // motor balance correction: logical only for 2 motor setup
    int otherMotor = (motor+1)%NUM_MOTORS; 
    long otherMotorError = (turning?-errors[otherMotor]:errors[otherMotor]);
    int motorDiffCorrection = (int) (K_DIFF * (errors[motor] - otherMotorError));
    if (motorDiffCorrection > MAX_CORRECTION)  motorDiffCorrection =  MAX_CORRECTION;
    if (motorDiffCorrection < -MAX_CORRECTION) motorDiffCorrection = -MAX_CORRECTION;
    
    return baseVal + motorDiffCorrection;
}

/*                                                                              
 * bool isZero()                                                                
 * Takes in the list of errors values and determines if all of them are within the 
 * margin set to be "zero". This helps prevent the encoders from causing the robot
 * to oscillate back and forth due to it not being able to hit exactly zero.    
 */                                                                             
bool isZero(long* errors){                                    
                                                                                
    for (int i = 0; i < NUM_MOTORS; i++){                                                
        if (abs(errors[i]) > ZERO_ERROR_MARGIN) return false;          
    }                                                                           
    return true;                                                                
}    

/*
 * void setMotor()
 * set the identified motor to the desired PWM value (uint_8)
 */
void setMotor(int m, int pwm){

    if (pwm > -MIN_SPEED && pwm < 0) pwm = -MIN_SPEED;
    if (pwm <  MIN_SPEED && pwm > 0) pwm =  MIN_SPEED;
    
    //negative on left motor is forwards
    pwm = (m==L) ? -pwm:pwm;

    //get motor pins
    int M_pin = (m==L) ? ML:MR;
    int E_pin = (m==L) ? EL:ER;
    

    //set direction and speed
    digitalWrite(M_pin, pwm > 0 ? HIGH:LOW);
    analogWrite (E_pin, abs(pwm));

}

/*
 * void stopMotors()
 * set both motors to zero speed
 */
void stopMotors(){
    setMotor(L,0);
    setMotor(R,0);
}

/*
 * void resetEncoderCounts()
 * Resets each of the encoder counts to zero
 */
void resetEncoderCounts(){
    for (int i = 0; i < NUM_MOTORS; i++) encCounts[i] = 0;
}

/*
 * void encoderCount()
 * Method indirectly used by the interrupts to determine the direction the wheel
 * is going based on which value changed and the state of the one that did
 * not change
 *
 */
void encoderCount(int enc, int type){
    int inc = (enc == R) ? -1 : 1;

    if (type == A){
      //low to high
      if (digitalRead(ENC_PINS[enc][A]) == HIGH)
          if (digitalRead( ENC_PINS[enc][B]) == LOW)  encCounts[enc]+=inc;
          else                                        encCounts[enc]-=inc;
      //high to low
      else
          if (digitalRead( ENC_PINS[enc][B]) == HIGH) encCounts[enc]+=inc;
          else                                        encCounts[enc]-=inc;
    }

    else if (type == B){
      //low to high
      if (digitalRead(ENC_PINS[enc][B]) == HIGH)
          if (digitalRead( ENC_PINS[enc][A]) == HIGH) encCounts[enc]+=inc;
          else                                        encCounts[enc]-=inc;
      //high to low
      else
          if (digitalRead( ENC_PINS[enc][A]) == LOW)  encCounts[enc]+=inc;
          else                                        encCounts[enc]-=inc;
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
