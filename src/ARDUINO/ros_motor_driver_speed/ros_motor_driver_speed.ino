#include <ros.h>
#include <PinChangeInterrupt.h>
//#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose2D.h>
#include "claw.h"


/*****************************************
 *       HARDWARE DEFINITIONS            * 
 *****************************************/
#define cameraServoPin 44
Servo cameraServo;
uint8_t cameraAngle = 0;

Claw claw;
uint8_t clawAngle = 10;

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

geometry_msgs::Pose2D robot_pose;

/**************************************** 
 *          GLOBAL VARIABLES            *
 ****************************************/
volatile double velocitySetpoints[NUM_MOTORS] = {0, 0};
volatile float velocities[NUM_MOTORS] = {0,0};
volatile long lastCounts[NUM_MOTORS] = {0, 0};
volatile float distanceSetpoint = 0;
volatile float angleSetpoint = 0;
volatile int moveState = CONST_VEL;
volatile bool newDriveCmd = false;
long deltaPosition[NUM_MOTORS];
float integralControlValue[NUM_MOTORS] = {0, 0};

/*****************************************
 *               ROS                     *
******************************************/
ros::NodeHandle_<ArduinoHardware, 25, 25, 1024, 256> nh;
//ros::NodeHandle nh;

void dcCallback(const geometry_msgs::Pose2D& moveCmd) {
    String logg = String(moveCmd.x) + String(", ") + String(moveCmd.y) + String(", ") + String(moveCmd.theta);
    nh.loginfo(logg.c_str());
    switch((uint8_t)moveCmd.y){
        case CONST_VEL:
            velocitySetpoints[L] = -moveCmd.theta;
            velocitySetpoints[R] = moveCmd.theta;
            velocitySetpoints[L] += moveCmd.x;
            velocitySetpoints[R] += moveCmd.x;
            break;
        case DRIVE_DIST:
            newDriveCmd = true;
            distanceSetpoint = moveCmd.x;
            break;
        case TURN_ANGLE:
            newDriveCmd = true;
            angleSetpoint = moveCmd.theta;
            break;
        default:
            break;
        }
     moveState = moveCmd.y;
}

void ccCallback(const std_msgs::UInt8& clawCmd) {
    clawAngle = clawCmd.data;
    claw.Servo_SetAngle(clawAngle);
}


void gcCallback(const std_msgs::Bool& gripCmd) {
    bool doClose = gripCmd.data;
    doClose ? claw.Gripper_Close():claw.Gripper_Open();
}

void camCallback(const std_msgs::UInt8& camCmd) {
    cameraAngle = camCmd.data;
    cameraServo.writeMicroseconds(DEG_TO_US(cameraAngle));
    //nh.loginfo("Got it!");
}


ros::Subscriber<geometry_msgs::Pose2D> dc ("drive_command", &dcCallback);
ros::Subscriber<std_msgs::UInt8>   cc ("claw_command",  &ccCallback);
ros::Subscriber<std_msgs::Bool> gc ("grip_command", &gcCallback);
ros::Subscriber<std_msgs::UInt8> cam("cam_command",   &camCallback);

ros::Publisher pose_pub("robot_pose", &robot_pose);

/*****************************************
 *  POSITION CONTROL & MOTOR FEEDBACK    *
 *****************************************/

// Control Values
#define SAMPLE_PERIOD           10
#define POSE_PUBLISH_RATE_MS    1000
#define ROS_UPDATE_RATE         1
#define DEG2RAD (PI/180)
// Velocity Controller
#define K_P                     19.4
#define K_I                     270 
// Distance Controller
#define K_P_DIST                1.1 //inches error -> rad/s
#define MAX_SPEED               3 //rad/s
#define MIN_SPEED               0.8 //rad/s
#define ZERO_ERROR_MARGIN       0.17 //inches until distance is considered achieved
#define K_DIFF                  1.5
#define MAX_CORRECTION          2
// Angle controller
#define K_P_ANG                 0.04 //degrees error -> rad/s
#define TURN_ZERO_ERROR_MARGIN  1 //degrees
#define TURN_ESS_GAIN     0.011 //degrees
#define SECONDS_PER_MILLISECOND 0.001
// robot specs
#define COUNTS_PER_REV       3200
#define WHEEL_DIAMETER       3.45 //inches
#define WHEEL_CIRC           10.84 //inches
#define ROBOT_WIDTH          7.63 //inches
#define TURN_CIRCUMFERENCE   (ROBOT_WIDTH*PI)
#define ANGLE_PER_REV        ((WHEEL_CIRC/TURN_CIRCUMFERENCE) * 360)
#define STRAIGHT_THRESH      0.5 //degrees
#define INCHES_PER_COUNT    (WHEEL_CIRC/COUNTS_PER_REV)
//IMU creation
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*
   void setup()
*/
void setup() {

    cameraServo.attach(cameraServoPin);
    cameraServo.writeMicroseconds(DEG_TO_US(0));
  
    claw.Claw_init();
  
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(dc);
    nh.subscribe(cc);
    nh.subscribe(gc);
    nh.subscribe(cam);
    nh.advertise(pose_pub);
    while (!nh.connected()) {
        nh.spinOnce();
    }


    robot_pose.x = 4.5*12;
    robot_pose.y = 4.5*12;
    robot_pose.theta = 90;

    /* Initialize the IMU */
    while (!bno.begin()){
        /* There was a problem detecting the BNO055 ... check your connections */
        nh.spinOnce();
        nh.loginfo("Imu not connected.");
        bno = Adafruit_BNO055(55);
    }
  
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(ENC_PINS[i][A], INPUT_PULLUP);
        pinMode(ENC_PINS[i][B], INPUT_PULLUP);
    }
  
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
   void loop()
*/
void loop() {
    switch(moveState){
        case CONST_VEL:
            velDrive();
            updatePosition();
            break;
        case DRIVE_DIST:
            if (newDriveCmd){
                newDriveCmd = false;
                distDrive();  
            }
            break;
        case TURN_ANGLE:
            if (newDriveCmd){
                newDriveCmd = false;
                turn();
            } 
            break;
        default:
            break;
        }
  rosUpdate();

}

void rosUpdate(){
    static long lastTime = millis();
    robot_pose.theta = getHeading();
    if (millis() - lastTime > POSE_PUBLISH_RATE_MS){
        pose_pub.publish(&robot_pose);
        lastTime = millis();
    }
    nh.spinOnce(); 
}

/*
   void velDrive()
   Takes in a velocity setpoint and drives forward at that speed.
*/
void velDrive() {
    unsigned long timeRef = millis();

    if (velocitySetpoints[L] == 0 && velocitySetpoints[R] == 0){
      stopMotors();
      return;
    }
    int motorVal[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        float velSetpoint = velocitySetpoints[i];
        motorVal[i] = findMotorPWMSetpoint(velSetpoint, i);
    }
    for (int i = 0; i < NUM_MOTORS; i++){
        setMotor(i, motorVal[i]);
    }
        
    //if (millis() > timeRef + SAMPLE_PERIOD) Serial.println("Error! Execution time too slow");
    while (millis() < timeRef + SAMPLE_PERIOD); // wait the sample period 
}

/*
 * Takes in the motor and angular velocity setpoint.
 * Implements a PI controller to convert the angular velocity setpoint to a PWM value and  
 * saturates the PWM value (-255 to 255).
 */
int findMotorPWMSetpoint(float angVelSetpoint, int motor) {
    updateVelocity(motor);
    float error = angVelSetpoint - velocities[motor];
    if (angVelSetpoint == 0) return 0;
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
                velocitySetpoints[i] = posErrorToAngVel(posError, i);
            }
        //changes motor output
        
        velDrive();
        rosUpdate();
        } while (!isZero(posError, false));
        stopMotors();
        // Delay for motor overshoot calculation
        delay(400);
    } while (!isZero(posError, false));
    stopMotors();
    robot_pose.x += distanceSetpoint*cos(DEG2RAD*getHeading());
    robot_pose.y += distanceSetpoint*sin(DEG2RAD*getHeading());
    
}

void turn(){
    resetEncoderCounts();
    float angError[NUM_MOTORS] = {0,0};
    float angleSetpoints[NUM_MOTORS] = {0,0};
    //CCW rotation, left is reverse, right is forward
    angleSetpoints[L] = -angleSetpoint - TURN_ESS_GAIN*angleSetpoint;
    angleSetpoints[R] = angleSetpoint + TURN_ESS_GAIN*angleSetpoint;
    do {
        do {
            for (int i = 0; i < NUM_MOTORS; i++){
                //find error in degrees
                angError[i] = angleSetpoints[i] - ((encCounts[i]*ANGLE_PER_REV)/(COUNTS_PER_REV));
                //changes velocity setpoint
                velocitySetpoints[i] = angErrorToAngVel(angError[i]);
            }
            //changes motor output
            velDrive(); //enforces set sample period
            rosUpdate();
        } while (!isZero(angError, true));
        stopMotors();
        // Delay for motor overshoot calculation
        delay(200);
    } while (!isZero(angError, true));
    stopMotors();
    robot_pose.theta = getHeading();
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
    volatile static double previousTheta = getHeading();
    robot_pose.theta = getHeading();
    
    float leftDelta = deltaPosition[L]*INCHES_PER_COUNT;
    float rightDelta = deltaPosition[R]*INCHES_PER_COUNT;
    double dTheta = boundAngle(robot_pose.theta - previousTheta);
    
    if(fabs(dTheta) <  STRAIGHT_THRESH){
        // Robot is going straight, update x and y based on the current heading
        robot_pose.x += leftDelta*cos(DEG2RAD*robot_pose.theta);
        robot_pose.y += rightDelta*sin(DEG2RAD*robot_pose.theta);
    }
    else if (fabs(velocities[L] + velocities[R]) < 0.25){
        // Robot is strictly turning, little change to robot position
    }
    else {
        // Robot is driving at an arc, approximate position as initial turn then straight path
        float arcLength = (leftDelta + rightDelta)/2;
        robot_pose.x += (arcLength*cos(DEG2RAD*robot_pose.theta));
        robot_pose.y += (arcLength*sin(DEG2RAD*robot_pose.theta));
    }
    previousTheta = robot_pose.theta;
    
}

// Keeps the angle within bounds of -180 to 180
float boundAngle(float angle){
    while(abs(angle) > 180){
        angle += angle > 0 ? -360 : 360;
    }
    return angle;
}


void updateVelocity(int motor) {
    long newEncCounts = encCounts[motor];
    deltaPosition[motor] = newEncCounts - lastCounts[motor];
    lastCounts[motor] = newEncCounts;
    velocities[motor] = (deltaPosition[motor] * 2 * PI) / (SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * COUNTS_PER_REV);
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

float posErrorToAngVel(float* errors, int motor){
    float angVel = (errors[motor]*K_P_DIST);
    //saturate vel values
    if (abs(angVel) > MAX_SPEED) angVel =  angVel > 0 ? MAX_SPEED : -MAX_SPEED;
    else if (abs(angVel) < MIN_SPEED) angVel =  angVel > 0 ? MIN_SPEED : -MIN_SPEED;
    
    // Do error correction to keep wheels consistent with eachother
    int otherMotor = (motor == L) ? R : L;
    float motorDiff = errors[motor] - errors[otherMotor];
    float motorDiffCorrection = K_DIFF * motorDiff;
    if (motorDiffCorrection > MAX_CORRECTION) motorDiffCorrection = MAX_CORRECTION;
    if (motorDiffCorrection < -MAX_CORRECTION) motorDiffCorrection = -MAX_CORRECTION;

    angVel += motorDiffCorrection;
    return angVel;
}

/*
   void setMotor()
   set the identified motor to the desired PWM value (uint_8)
*/
void setMotor(int m, int pwm) {
    //Serial.println("Motor: " + String(m) + ": " + String(pwm));
  
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
    integralControlValue[L] = 0;
    integralControlValue[R] = 0;
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

double getHeading(){
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    double angle = (360 - orientationData.orientation.x) + 90;
    angle = boundAngle(angle);
    return angle;
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
