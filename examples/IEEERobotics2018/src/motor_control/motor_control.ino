//software interrupts library
#include <PinChangeInt.h>
//i2c library
#include <Wire.h>

const int I2C_ADDR = 0x42;

const unsigned long COUNTS_PER_REV = 3200;
const unsigned int  MOVING_AVG_SIZE = 25;

const float DISTANCE_PER_REV = 3.14159265 * 4.05; //pi * wheel diameter
const float ROBOT_LENGTH = 10.75;

//control info
const float K_P = 1/80.0;
const int ZERO_ERROR_MARGIN = 280;
const float ERROR_THRESHOLD_POS = 0.25;
const float ERROR_THRESHOLD_ROT = 3.0;
int  MIN_MOTOR_SPEED = 80;
int  MAX_MOTOR_SPEED = 80;
int  TURN_SPEED      = 130;
int  CONFORM_SPEED   = -50;

//positional info
enum POS_INFO{X, Y, R, NUM_AXIS};
volatile float current_pos[NUM_AXIS];
volatile float target_pos[NUM_AXIS];

//encoder info
volatile long enc_counts[4];
volatile float gyro_rot = 0;
volatile int dir;
unsigned int ENC_PINS[4][2] = { {A8, A9 },
                                {10, 11},
                                {A10,A11},
                                {12, 13} };


//motor pins
unsigned int PWM_PINS[4] = {2, 3, 4, 5};
unsigned int DIR_PINS[4] = {23, 22, 25, 24};

//motor distinction
enum ENC {FL, FR, BL, BR};
enum TYPE {A, B};

//communication info
enum COMMANDS {GET_TARG_POS, GET_CUR_POS, GET_GYRO_ROT, FORCE_STOP};
byte completed_movement = 0;

int posUpdated = false;
int force_stop = false;

void setup() {

    Serial.begin(9600);
    
    Wire.begin(I2C_ADDR);
    Wire.onReceive(getPosition);
    Wire.onRequest(checkDone);
  
    for (int i = 0; i < 4; i++){
        pinMode(ENC_PINS[i][A], INPUT_PULLUP);
        pinMode(ENC_PINS[i][B], INPUT_PULLUP);
  
        pinMode(DIR_PINS[i], OUTPUT);
        pinMode(PWM_PINS[i], OUTPUT);
    }
    
    PCintPort::attachInterrupt(ENC_PINS[FL][A], FLA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FL][B], FLB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FR][A], FRA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[FR][B], FRB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BL][A], BLA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BL][B], BLB_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BR][A], BRA_changed,  CHANGE);
    PCintPort::attachInterrupt(ENC_PINS[BR][B], BRB_changed,  CHANGE);

    current_pos[X] = 48.0;
    current_pos[Y] = 6.0;
    current_pos[R] = 90.0;

}

void loop() {

    //change in position was detected
    if(posUpdated){
        posUpdated = false;      
        gotoTarget();
    }

    if (force_stop){
        force_stop = false;
        stopMotors();
        while(true){}
    }
  
}

void gotoTarget(){
    float curToTarg1[3];
    curToTarg1[X] = target_pos[X]-current_pos[X];
    curToTarg1[Y] = target_pos[Y]-current_pos[Y];
    curToTarg1[R] = degrees(atan2(curToTarg1[Y], curToTarg1[X]));

    float curToTarg2[3];
    curToTarg2[X] = current_pos[X]-target_pos[X];
    curToTarg2[Y] = current_pos[Y]-target_pos[Y];
    curToTarg2[R] = degrees(atan2(curToTarg2[Y], curToTarg2[X]));
    
    //turn to target pos
    float turnAngleToTarget1 = getTurnAngle(current_pos[R], curToTarg1[R]);
    float turnAngleToTarget2 = getTurnAngle(current_pos[R], curToTarg2[R]);

    float distToTarget = sqrt(pow(curToTarg1[X],2) + pow(curToTarg1[Y], 2));

    Serial.println(abs(turnAngleToTarget1));
    Serial.println(abs(turnAngleToTarget2));
    float turnAngleToAlign;
    if (abs(turnAngleToTarget1) <= abs(turnAngleToTarget2)){
      if (distToTarget != 0){
          turn(turnAngleToTarget1);
          drive(distToTarget);
          turnAngleToAlign = getTurnAngle(curToTarg1[R], target_pos[R]);
      }
      else {
          turnAngleToAlign = getTurnAngle(current_pos[R] , target_pos[R]);  
      }
      turn(turnAngleToAlign);
      
    }
    else {
      if (distToTarget != 0){
          turn(turnAngleToTarget2);
          drive(-distToTarget);
          turnAngleToAlign = getTurnAngle(curToTarg2[R], target_pos[R]);
      }
      else {
          turnAngleToAlign = getTurnAngle(current_pos[R] , target_pos[R]);  
      }
      turn(turnAngleToAlign); 
      
    }


    completed_movement = 1;
    current_pos[X] = target_pos[X];
    current_pos[Y] = target_pos[Y];
    current_pos[R] = target_pos[R];
    
}

/*
 * void drive()
 * Takes in a distance in inches and drives forwards/backwards until value is reached
 */
void drive(float distance){

    resetEncoderCounts();
    
    long start_rot  = gyro_rot;
    
    long goals[4];
    for (int i = 0; i < 4; i++){
        goals[i] = (long) (distance * COUNTS_PER_REV / DISTANCE_PER_REV); 
    }

    long errors[4][MOVING_AVG_SIZE];
    int replaceIndex = 0;
    bool filled = false;
    
    do { 
        if (force_stop) loop();
        for (int i = 0; i < 4; i++){
            errors[i][replaceIndex] = enc_counts[i] - goals[i];
            if (!filled) continue;
   
            int motorVal = errorToMotorOut(K_P, errors[i][replaceIndex]);
            
            setMotor(i, motorVal);
        }

        if (replaceIndex == MOVING_AVG_SIZE-1) filled = true;
        replaceIndex = (replaceIndex + 1) % MOVING_AVG_SIZE;   
        
    } while (!filled || !isZero(errors));
       
    stopMotors();
}

/*
 * void turn()
 * Takes in an angle in degrees and turns CW/CCW until value is reached
 */
void turn(float angle){

    float startBack, endBack;
    if (abs(angle) == 45){
        startBack = -ROBOT_LENGTH * abs(angle)/135.0;
        endBack   = -ROBOT_LENGTH * abs(angle)/210.0;
    }
    else if (abs(angle) == 90){
        startBack = -ROBOT_LENGTH * abs(angle)/210.0;
        endBack   = -ROBOT_LENGTH * abs(angle)/135.0; 
    }
  
    
    drive(-ROBOT_LENGTH * abs(angle)/210.0);
    delay(500);

    bool reverse = false;

    long start_rot  = gyro_rot;
    long target_rot = start_rot + angle;
    
    bool corrected = false;
    while ( !corrected){
        bool filled = false;
        int replaceIndex = 0;
        long errors[MOVING_AVG_SIZE];
        do {
            if (force_stop) loop();
          
            errors[replaceIndex] = gyro_rot - target_rot;
            if (replaceIndex == MOVING_AVG_SIZE-1) filled = true;
            replaceIndex = (replaceIndex + 1) % MOVING_AVG_SIZE;
            if (!filled) continue;
            
            long avg = average(errors);

            if (!reverse){
                setMotor(FL, avg > 0 ? TURN_SPEED     : CONFORM_SPEED  );
                setMotor(BL, avg > 0 ? TURN_SPEED + 30: CONFORM_SPEED  );
                setMotor(FR, avg > 0 ? CONFORM_SPEED  : TURN_SPEED     ); 
                setMotor(BR, avg > 0 ? CONFORM_SPEED  : TURN_SPEED + 30); 
            }
            else{
                setMotor(FL, avg < 0 ? -TURN_SPEED - 30  : -CONFORM_SPEED );
                setMotor(BL, avg < 0 ? -TURN_SPEED: -CONFORM_SPEED  );
                setMotor(FR, avg < 0 ? -CONFORM_SPEED  : -TURN_SPEED -30 ); 
                setMotor(BR, avg < 0 ? -CONFORM_SPEED  : -TURN_SPEED ); 
            }
            
        } while (abs(gyro_rot - target_rot) > ERROR_THRESHOLD_ROT);
        stopMotors();
        delay(250);  
        corrected = abs(gyro_rot - target_rot) <= ERROR_THRESHOLD_ROT;
        if (!corrected){reverse = !reverse;}
    }
    
    delay(500);
    drive(-ROBOT_LENGTH * abs(angle)/135.0);
    stopMotors();
  
}
/*
 * int errorToMotorOut()
 * Takes in an error and gain value and converts it to a motor output with an 
 * upper bound check of max speed
 */
int errorToMotorOut(float gain, long error){
    //no error
    //if (abs(error) <= ZERO_ERROR_MARGIN) return 0;

    //motor level with base power + error * gain
    int motorOut = gain * -error + (error <= 0 ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED);

    //bound check
    if (motorOut > MAX_MOTOR_SPEED) return MAX_MOTOR_SPEED;
    if (motorOut < -MAX_MOTOR_SPEED) return -MAX_MOTOR_SPEED;

    return motorOut;
}

/*
 * int setMotor()
 * Takes in a motor index and pwm value and corrects it to properly set the direction
 * pin and the abs(pwm) as the pwm
 */
void setMotor(int m, int pwm){
    //set direction
    digitalWrite(DIR_PINS[m], pwm > 0 ? HIGH:LOW);

    //set speed
    if (pwm < 0)   pwm = -pwm;
    analogWrite(PWM_PINS[m], pwm);
}

void stopMotors(){
    
    for(int i = 0; i < 4; i++){
        setMotor(i, 0);  
    } 
}

/*
 * bool isZero()
 * Takes in the list of errors values and determines if all of them are within the 
 * margin set to be "zero". This helps prevent the encoders from causing the robot
 * to oscillate back and forth due to it not being able to hit exactly zero. 
 */
bool isZero(long errors[][MOVING_AVG_SIZE]){

    bool zero = true;
    for (int i = 0; i < 4; i++){
        if (abs(average(errors[i])) > ZERO_ERROR_MARGIN) zero = false;
    }
    return zero;
}

long average(long* errors){
    double sum = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++){
        sum += errors[i];
    }
    return (long)(sum / MOVING_AVG_SIZE);
  
}

/*
 * void resetEncoderCounts()
 * Resets each of the encoder counts to zero 
 */
void resetEncoderCounts(){
    for (int i = 0; i < 4; i++) enc_counts[i] = 0;  
}


/*
 * void encoderCount()
 * Method indirectly used by the interrupts to determine the direction the wheel
 * is going based on which value changed and the state of the one that did 
 * not change
 * 
 */
void encoderCount(int enc, int type){
    int inc = (enc == FR || enc == BR) ? 1 : -1;
  
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
void FLA_changed(){encoderCount(FL, A);}
void FLB_changed(){encoderCount(FL, B);}
void FRA_changed(){encoderCount(FR, A);}
void FRB_changed(){encoderCount(FR, B);}
void BLA_changed(){encoderCount(BL, A);}
void BLB_changed(){encoderCount(BL, B);}
void BRA_changed(){encoderCount(BR, A);}
void BRB_changed(){encoderCount(BR, B);}


void getPosition(int num_bytes) {
    // The smbus implementation sends over a command and the number of bytes it has in its payload
    // before it actually, dir sends over the real data
    int cmd = Wire.read();
    int bytes = Wire.read();
    
    // Do a sanity check to make sure that the data received follows the correct format
    if (num_bytes == bytes + 2) {
        
        if (cmd == GET_TARG_POS) {
          posUpdated = true;  
          completed_movement = 0;
            for(int i = 0; i < 3; i++)
                target_pos[i] = i2cGetInt() / 100.0;
        }
        else if (cmd == GET_CUR_POS) {
          posUpdated = true;
          completed_movement = 0;
            for (int i = 0; i < 3; i++)
                current_pos[i] = i2cGetInt() / 100.0;
        }
        else if (cmd == GET_GYRO_ROT){
            gyro_rot = i2cGetInt() / 100.0;
        }
        else if (cmd == FORCE_STOP){
            i2cGetInt();
            force_stop = true;  
        }
        else {
            dumpData();
        }
    }
    else {
        // We have an unexpected message, throw it out.
        dumpData();   
    }
    
}

void checkDone(){
    Wire.write(completed_movement);
}


// Reads the next 2 bytes from the i2c bus and splices them together to make a signed 16-bit integer.
int i2cGetInt() {
    return ((Wire.read() << 8) | Wire.read());
}

void dumpData() {
    while (Wire.available()) Wire.read();
}

float dot_product(volatile float *a, volatile float *b,int size)
{
    float dp = 0.0f;
    for (int i=0;i<size;i++)
        dp += a[i] * b[i];
    return dp;
}

float getTurnAngle(float s, float e){
    float alpha = e - s;
    float minAngle = abs(alpha);
    float angle = alpha;
    float beta = e - s + 360;
    if (abs(beta) < minAngle){
        minAngle = abs(beta);
        angle = beta;
    }
    float gamma = e - s - 360;
    if (abs(gamma) < minAngle){
        minAngle = abs(gamma);
        angle = gamma;
    }
    return angle;
}

