#include <PinChangeInterrupt.h>
#include <Servo.h>
#include "claw.h"

//#include <ros.h>
//#include <std_msgs/UInt8.h>
//#include <geometry_msgs/Pose2D.h>

/*****************************************
  HARDWARE DEFINITIONS
*****************************************/
#define cameraServoPin 11
Servo cameraServo;
uint8_t cameraAngle = 0;

Claw claw;

//helper enums
enum MOTOR_IDS {L, R, NUM_MOTORS};
enum ENC_TYPE {A, B};
enum STATE {CONST_VEL, DRIVE_DIST, TURN_ANGLE};

//M is mode (direction), E is PWM (speed)
enum MOTOR_PINS {ML = 7, EL = 9, MR = 8, ER = 10};

// Encoder info
volatile long encCounts[2];
unsigned int ENC_PINS[NUM_MOTORS][2] = { { 50,  51},   // Left
                                         { 52,  53} }; // Right

//geometry_msgs::Pose2D robot_pose;

// Global variables
double velocitySetpoint[NUM_MOTORS] = {0, 0};
long lastCounts[NUM_MOTORS] = {0, 0};
float distanceSetpoint = 0;
float angleSetpoint = 0;
int moveState = CONST_VEL;

/*****************************************
  ROS
*****************************************/

/*ros::NodeHandle_<ArduinoHardware, 5, 5, 1024, 512> nh;

void dcCallback(const geometry_msgs::Pose2D& moveCmd) {
  velocitySetpoint[L] = -moveCmd.theta;
  velocitySetpoint[R] = moveCmd.theta;
  velocitySetpoint[L] += moveCmd.x;
  velocitySetpoint[R] += moveCmd.x;
}

#define PICKUP 0
#define PUTDOWN 1
void ccCallback(const std_msgs::UInt8& clawCmd) {
  uint8_t cmd = clawCmd.data;
  nh.loginfo(String(cmd).c_str());
  if (cmd == PICKUP) {
    claw.Gripper_Close();
    claw.Servo_SetLevel();
  }
  else if (cmd == PUTDOWN) {
    claw.Servo_SetMin();
    claw.Gripper_Open();
  }
}

void camCallback(const std_msgs::UInt8& camCmd) {
  cameraAngle = camCmd.data;
  cameraServo.write(cameraAngle);
  nh.loginfo("Got it!");
}

ros::Subscriber<geometry_msgs::Pose2D> dc ("drive_command", &dcCallback);
ros::Subscriber<std_msgs::UInt8>   cc ("claw_command",  &ccCallback);
ros::Subscriber<std_msgs::UInt8> cam("cam_command",   &camCallback);

ros::Publisher pose_pub("robot_pose", &robot_pose);
*/
/*****************************************
  POSITION CONTROL & MOTOR FEEDBACK
*****************************************/

// Control Values
#define SAMPLE_PERIOD           10
// Velocity Controller
#define K_P                     19.4
#define K_I                     270 
//#define K_P                     4
//#define K_I                     100
// Distance Controller
#define K_P_DIST                1.1 //inches error -> rad/s
#define MAX_SPEED               7 //rad/s
#define MIN_SPEED               0.8 //rad/s
#define ZERO_ERROR_MARGIN       0.17 //inches until distance is considered achieved
// Angle controller
#define K_P_ANG                 0.04 //degrees error -> rad/s
#define TURN_ZERO_ERROR_MARGIN  1 //degrees
#define TURN_ESS_ADJUSTMENT     1 //degrees

#define SECONDS_PER_MILLISECOND 0.001
// robot specs
#define COUNTS_PER_REV       3200
#define WHEEL_DIAMETER       3.45 //inches
#define WHEEL_CIRC           10.84 //inches
#define ROBOT_WIDTH          7.63 //inches
#define TURN_CIRCUMFERENCE   (ROBOT_WIDTH*PI)
#define ANGLE_PER_REV        ((WHEEL_CIRC/TURN_CIRCUMFERENCE) * 360)
#define STRAIGHT_THRESH      20
#define INCHES_PER_COUNT    (COUNTS_PER_REV/WHEEL_CIRC)

long deltaPosition[NUM_MOTORS];
float integralControlValue[NUM_MOTORS] = {0, 0};
/*
   void setup()
*/
void setup() {

  Serial.begin(115200);


  /*robot_pose.x = 3.5;
  robot_pose.y = 3.5;
  robot_pose.theta = 90;*/

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(ENC_PINS[i][A], INPUT_PULLUP);
    pinMode(ENC_PINS[i][B], INPUT_PULLUP);
  }

  pinMode(ML, OUTPUT);
  pinMode(EL, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(ER, OUTPUT);

  cameraServo.attach(cameraServoPin);
  cameraServo.write(0);

  claw.Claw_init();

  // attach interrupts to four encoder pins
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][A]), LA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[L][B]), LB_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][A]), RA_changed,  CHANGE);
  attachPCINT(digitalPinToPCINT(ENC_PINS[R][B]), RB_changed,  CHANGE);

  /*nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.subscribe(dc);
  nh.subscribe(cc);
  nh.subscribe(cam);
  nh.advertise(pose_pub);
  while (!nh.connected()) {
    nh.spinOnce();
  }*/
}

/*
   void loop()
*/
void loop() {
    //pose_pub.publish(&robot_pose);
    //nh.spinOnce();

    /*switch (moveState) {
        case CONST_VEL:
            velDrive();
        case DRIVE_DIST:
            distDrive();
        case TURN_ANGLE:
            turn();
    }*/
    for (int i = 0; i < 5; i++){
        distanceSetpoint = 16;
        distDrive();
        delay(1000);
        angleSetpoint = 90;
        turn();
        delay(1000);
    }
    delay(5000);
    /*velocitySetpoint[L] = 2;
    velocitySetpoint[R] = 2;
    velDrive();*/
    
    //updatePosition();
    
}

/*
   void velDrive()
   Takes in a velocity setpoint and drives forward at that speed.
*/
void velDrive() {
    unsigned long timeRef = millis();
    if (velocitySetpoint[0] != 0) {
        int motorVal[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            float velSetpoint = velocitySetpoint[i];
            motorVal[i] = findMotorPWMSetpoint(velSetpoint, i);
            //setMotor(i, motorVal);
        }
        for (int i = 0; i < NUM_MOTORS; i++){
            setMotor(i, motorVal[i]);
        }
    }
    else stopMotors();
    if (millis() > timeRef + SAMPLE_PERIOD) Serial.println("Error! Execution time too slow");
    while (millis() < timeRef + SAMPLE_PERIOD) {
        //nh.spinOnce();
    }
}

/*
 * Takes in the motor and angular velocity setpoint.
 * Implements a PI controller to convert the angular velocity setpoint to a PWM value and  
 * saturates the PWM value (-255 to 255).
 */
int findMotorPWMSetpoint(float angVelSetpoint, int motor) {
    float error = angVelSetpoint - getVelocity(motor);
    integralControlValue[motor] += SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * error;
    int PWMSetpoint = (int)(error * K_P + integralControlValue[motor] * K_I);
  
    //saturate PWM value
    if (abs(PWMSetpoint) > 255) {
      return PWMSetpoint > 0 ? 255 : -255;
    }
    else return PWMSetpoint;
}

/*
 * Drives the set distance (stored in float distance). 
 */
void distDrive(){
    resetEncoderCounts();
    float posError[NUM_MOTORS] = {0,0};
    do{
        do{
            for (int i = 0; i < NUM_MOTORS; i++){
                //find error in inches
                posError[i] = distanceSetpoint - ((encCounts[i]*WHEEL_CIRC)/COUNTS_PER_REV);
                //changes velocity setpoint
                velocitySetpoint[i] = posErrorToAngVel(posError[i]);
            }
        //changes motor output
        velDrive();
        } while (!isZero(posError, false));
        stopMotors();
        // Delay for motor overshoot calculation
        delay(400);
    } while (!isZero(posError, false));
    stopMotors();
}

void turn(){
    resetEncoderCounts();
    float angError[NUM_MOTORS] = {0,0};
    float angleSetpoints[NUM_MOTORS] = {0,0};
    float angleSet = angleSetpoint + (angleSetpoint > 0 ? TURN_ESS_ADJUSTMENT : -TURN_ESS_ADJUSTMENT);
    if (angleSetpoint > 0) {
        //CCW rotation, left is reverse, right is forward
        angleSetpoints[L] = -angleSet;
        angleSetpoints[R] = angleSet;
    } else {
        //CW rotation, right is reverse, left is forward
        angleSetpoints[L] = angleSet;
        angleSetpoints[R] = -angleSet;
    }
    do {
        do {
            for (int i = 0; i < NUM_MOTORS; i++){
                //find error in degrees
                angError[i] = angleSetpoints[i] - ((encCounts[i]*ANGLE_PER_REV)/(COUNTS_PER_REV));
                //changes velocity setpoint
                velocitySetpoint[i] = angErrorToAngVel(angError[i]);
            }
            //changes motor output
            velDrive(); //enforces set sample period
        } while (!isZero(angError, true));
        stopMotors();
        // Delay for motor overshoot calculation
        delay(200);
    } while (!isZero(angError, true));
    stopMotors();
}

/*
 * Takes in the angle error in degrees (float) and applies a gain to return the ang vel setpoint (float).
 * The proportional control gain is set by K_P_ANG.
 */
float angErrorToAngVel(float error){
    float angVel = (error*K_P_ANG);
    if (abs(angVel) > MAX_SPEED) {
        return angVel > 0 ? MAX_SPEED : -MAX_SPEED;
    }
    else if (abs(angVel) < MIN_SPEED) {
        return angVel > 0 ? MIN_SPEED : -MIN_SPEED;
    }
    else return angVel;
}

/*
 * Takes in an array and checks it against the ZERO_ERROR_MARGIN constant. 
 * The function returns true if the values in the array are less than the
 * error margin. 
 */
bool isZero(float* errors, bool turning){                                                                                                               
    for (int i = 0; i < NUM_MOTORS; i++){ 
        float margin = turning ? TURN_ZERO_ERROR_MARGIN : ZERO_ERROR_MARGIN;                                               
        if (abs(errors[i]) > margin) return false;          
    }                                                                           
    return true;                                                                
} 

  
void updatePosition(){
    float leftDelta = deltaPosition[L]*INCHES_PER_COUNT, rightDelta = deltaPosition[R]*INCHES_PER_COUNT;
    if(fabs((leftDelta - rightDelta)) <  STRAIGHT_THRESH){
        // Robot is going straight, update x and y, no change to angle
        /*robot_pose.x += leftDelta*cos(robot_pose.theta);
        robot_pose.y += rightDelta*sin(robot_pose.theta);*/
    }
    else {
        float turnRadius = ROBOT_WIDTH*(leftDelta + rightDelta)/(2*(rightDelta - leftDelta));
        float dTheta = (rightDelta - leftDelta)/ROBOT_WIDTH;
        /*robot_pose.x += (turnRadius*sin(dTheta + robot_pose.theta) - radius*sin(robot_pose.theta);
        robot_pose.y += (turnRadius*cos(dTheta + robot_pose.theta) - radius*cos(robot_pose.theta);
        robot_pose.theta = boundAngle(robot_pose.theta + dTheta);*/
    }
}

// Keeps the angle within bounds of -360 to +360
float boundAngle(float angle){
    while(abs(angle > 360)){
        angle += angle > 0 ? -360 : 360;
    }
}


float getVelocity(int motor) {
    long newEncCounts = encCounts[motor];
    deltaPosition[motor] = newEncCounts - lastCounts[motor];
    lastCounts[motor] = newEncCounts;
    float velocity = (deltaPosition[motor] * 2 * PI) / (SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * COUNTS_PER_REV);
    return velocity;
}

void printErrors(long* errors) {

  for (int i = 0; i < NUM_MOTORS; i++) {
    //Serial.print(errors[i]);
    //nh.loginfo(String(errors[i]).c_str());
    if (i != NUM_MOTORS - 1) {
      //Serial.print(", ");
      //nh.loginfo(", ");
    }

  }
  ////Serial.println("");
  //nh.loginfo("--------");
}

float posErrorToAngVel(float error){
    float angVel = (error*K_P_DIST);
    if (abs(angVel) > MAX_SPEED) {
        return angVel > 0 ? MAX_SPEED : -MAX_SPEED;
    }
    else return angVel;
}

/*
   void setMotor()
   set the identified motor to the desired PWM value (uint_8)
*/
void setMotor(int m, int pwm) {
    Serial.println("Motor: " + String(m) + ": " + String(pwm));
  
    //negative on left motor is forwards
    pwm = (m == L) ? -pwm : pwm;
  
    //get motor pins
    int M_pin = (m == L) ? ML : MR;
    int E_pin = (m == L) ? EL : ER;
  
    //set direction and speed
    digitalWrite(M_pin, pwm > 0 ? HIGH : LOW);
    analogWrite (E_pin, abs(pwm));
}

/*
   void stopMotors()
   set both motors to zero speed
*/
  void stopMotors() {
    setMotor(L, 0);
    setMotor(R, 0);
}

/*
   void resetEncoderCounts()
   Resets each of the encoder counts to zero
*/
void resetEncoderCounts() {
    for (int i = 0; i < NUM_MOTORS; i++){
      encCounts[i] = 0;
      lastCounts[i] = 0;
    }
}

/*
   void encoderCount()
   Method indirectly used by the interrupts to determine the direction the wheel
   is going based on which value changed and the state of the one that did
   not change

*/
void encoderCount(int enc, int type) {
  int inc = (enc == R) ? -1 : 1;

  if (type == A) {
    //low to high
    if (digitalRead(ENC_PINS[enc][A]) == HIGH)
      if (digitalRead( ENC_PINS[enc][B]) == LOW)  encCounts[enc] += inc;
      else                                        encCounts[enc] -= inc;
    //high to low
    else if (digitalRead( ENC_PINS[enc][B]) == HIGH) encCounts[enc] += inc;
    else                                        encCounts[enc] -= inc;
  }

  else if (type == B) {
    //low to high
    if (digitalRead(ENC_PINS[enc][B]) == HIGH)
      if (digitalRead( ENC_PINS[enc][A]) == HIGH) encCounts[enc] += inc;
      else                                        encCounts[enc] -= inc;
    //high to low
    else if (digitalRead( ENC_PINS[enc][A]) == LOW)  encCounts[enc] += inc;
    else                                        encCounts[enc] -= inc;
  }
}

/*
   A series of void arg void callback methods that pass arguments needed to
   the actual interrupt function since interrupts do not allow argument passing
*/
void LA_changed() {
  encoderCount(L, A);
}
void LB_changed() {
  encoderCount(L, B);
}
void RA_changed() {
  encoderCount(R, A);
}
void RB_changed() {
  encoderCount(R, B);
}
