#include <ros.h>
#include <PinChangeInterrupt.h>
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
enum STATE {CONST_VEL, DRIVE_DIST, TURN_ANGLE, POSITION_OVERRIDE, 
            DROP_BLOCK, PICKUP_BLOCK, LOOK_IN_MOTHERSHIP};

//M is mode (direction), E is PWM (speed)
enum MOTOR_PINS {ML = 7, EL = 9, MR = 8, ER = 10};
#define DRIVE_BUSY 46
#define STOP_PIN A0

// Encoder info
volatile long encCounts[2];
unsigned int ENC_PINS[NUM_MOTORS][2] = { { 50,  51},   // Left
                                         { 52,  53} }; // Right

#define SEARCH_LIGHT_PIN 13
#define LIGHT_BRIGHTNESS 75

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
volatile long deltaPosition[NUM_MOTORS];
volatile float integralControlValue[NUM_MOTORS] = {0, 0};
volatile float lastTurnError[NUM_MOTORS] = {0,0};
volatile float turnIntegral[NUM_MOTORS] = {0,0};
bool turboAvailible = false;
bool turning = false;

/*****************************************
 *               ROS                     *
******************************************/
ros::NodeHandle_<ArduinoHardware, 25, 25, 1024, 512> nh;
//ros::NodeHandle nh;

void dcCallback(const geometry_msgs::Pose2D& moveCmd) {
    //String logg = String(moveCmd.x) + String(", ") + String(moveCmd.y) + String(", ") + String(moveCmd.theta);
    //nh.loginfo(logg.c_str());
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
        case POSITION_OVERRIDE:
            robot_pose.x = moveCmd.x;
            robot_pose.y = moveCmd.theta;
            break;
        case DROP_BLOCK:
            newDriveCmd = true;
            distanceSetpoint = moveCmd.x;
            angleSetpoint = moveCmd.theta;
            break;
        case PICKUP_BLOCK:
            newDriveCmd = true;
            break;
        case LOOK_IN_MOTHERSHIP:
            newDriveCmd = true;
            distanceSetpoint = moveCmd.x;
            break;
        default:
            break;
        }
     if (newDriveCmd) digitalWrite(DRIVE_BUSY, HIGH);
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

#define POSE_PUBLISH_RATE_MS    250
ros::Subscriber<geometry_msgs::Pose2D> dc ("drive_command", &dcCallback);
ros::Subscriber<std_msgs::UInt8>   cc ("claw_command",  &ccCallback);
ros::Subscriber<std_msgs::Bool> gc ("grip_command", &gcCallback);
ros::Subscriber<std_msgs::UInt8> cam("cam_command",   &camCallback);

ros::Publisher pose_pub("robot_pose", &robot_pose);

void rosUpdate(bool busy){
    nh.spinOnce(); 
    if (analogRead(STOP_PIN) > 250) do_stop();
    static long lastTime = millis();
    //robot_pose.theta = getHeading();
    if (!busy && millis() - lastTime > POSE_PUBLISH_RATE_MS){
        pose_pub.publish(&robot_pose);
        lastTime = millis();
        digitalWrite(DRIVE_BUSY, LOW);
    }
}



/*****************************************
 *  POSITION CONTROL & MOTOR FEEDBACK    *
 *****************************************/

// Control Values
#define SAMPLE_PERIOD           10
#define ROS_UPDATE_RATE         1
#define DEG2RAD                 (PI/180)
#define OVERSHOOT_DELAY_MS      100
#define MOVE_TIMEOUT            1q000
// Velocity Controller
#define K_P                     19
#define K_I                     217 
#define K_P_TURN                70
//#define K_I                     0
// Distance Controller
#define K_P_DIST                1.1 //inches error -> rad/s
#define MAX_SPEED               6 //rad/s
#define TURBO                   2
#define TURBO_SETPOINT          0.5
#define MAX_TURN_SPEED          10 //rad/s
#define MAX_REVERSE_SPEED       4 //rad/s, slower because the wheel is crappy
#define MIN_SPEED               1.0 //rad/s
#define ZERO_ERROR_MARGIN       0.25 //inches until distance is considered achieved
#define K_DIFF                  1.5
#define MAX_CORRECTION          2
// Angle controller
#define K_P_ANG                 0.3 //degrees error -> rad/s
#define K_D_ANG                 0
#define K_I_ANG                 0.02
#define TURN_ZERO_ERROR_MARGIN  1 //degrees
//#define TURN_ESS_GAIN           0.011 //degrees
#define TURN_ESS_GAIN           0 //degrees
#define SECONDS_PER_MILLISECOND 0.001
// robot specs
#define COUNTS_PER_REV          3200
#define WHEEL_DIAMETER          3.45 //inches //3.45 orig
#define WHEEL_CIRC              10.84 //inches
#define ROBOT_WIDTH             7.63 //inches
#define TURN_CIRCUMFERENCE      (ROBOT_WIDTH*PI)
#define ANGLE_PER_REV           ((WHEEL_CIRC/TURN_CIRCUMFERENCE) * 360)
#define STRAIGHT_THRESH         0.5 //degrees
#define INCHES_PER_COUNT        (WHEEL_CIRC/COUNTS_PER_REV)

#define CLAW_PICKUP_ANGLE       40
//IMU creation
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*
   void setup()
*/
void setup() {
    cameraServo.attach(cameraServoPin);
    cameraServo.writeMicroseconds(DEG_TO_US(0));
    
    pinMode(SEARCH_LIGHT_PIN, OUTPUT);
    analogWrite(SEARCH_LIGHT_PIN, LIGHT_BRIGHTNESS);
    
    pinMode(DRIVE_BUSY, OUTPUT);
    digitalWrite(DRIVE_BUSY, HIGH);

    pinMode(STOP_PIN, INPUT);
  
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

    // Robot initial conditions
    robot_pose.x = 4.5*12;
    robot_pose.y = 4.5*12;
    robot_pose.theta = 90;

    /* Initialize the IMU */
    bool flash = true;
    while (!bno.begin()){
        flash ^= 1;
        /* There was a problem detecting the BNO055 ... check your connections */
        nh.spinOnce();
        nh.loginfo("Imu not connected.");
        bno = Adafruit_BNO055(55);
        analogWrite(SEARCH_LIGHT_PIN, flash ? LIGHT_BRIGHTNESS:0);
        delay(250);
    }
  
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(ENC_PINS[i][A], INPUT_PULLUP);
        pinMode(ENC_PINS[i][B], INPUT_PULLUP);
    }
  
    pinMode(ML, OUTPUT);
    pinMode(EL, OUTPUT);
    pinMode(MR, OUTPUT);
    pinMode(ER, OUTPUT);
    
    digitalWrite(DRIVE_BUSY, LOW);
  
    // attach interrupts to four encoder pins
    attachPCINT(digitalPinToPCINT(ENC_PINS[L][A]), LA_changed,  CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_PINS[L][B]), LB_changed,  CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_PINS[R][A]), RA_changed,  CHANGE);
    attachPCINT(digitalPinToPCINT(ENC_PINS[R][B]), RB_changed,  CHANGE);
    //attachPCINT(digitalPinToPCINT(STOP_PIN), do_stop,  CHANGE);
}

/*
   void loop()
*/
void loop() {

    if (newDriveCmd) digitalWrite(DRIVE_BUSY, HIGH);
  
    switch(moveState){
        case CONST_VEL:
        case POSITION_OVERRIDE:
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
        case DROP_BLOCK:
            if (newDriveCmd){
                newDriveCmd = false;
                stopMotors();
                // Turn the specified angle
                turn();
                // Lift the claw
                claw.Servo_SetAngle(CLAW_PICKUP_ANGLE);
                // Drive forward
                distDrive();
                // Drop off block
                claw.Gripper_Open();
                // Backup
                distanceSetpoint = -distanceSetpoint;
                distDrive();
            }
            break;
        case PICKUP_BLOCK:
            if (newDriveCmd){
                newDriveCmd = false;
                stopMotors();
                claw.Servo_SetAngle(5);
                claw.Gripper_Close();
                claw.Servo_SetAngle(CLAW_PICKUP_ANGLE);
                cameraServo.writeMicroseconds(DEG_TO_US(40));
            }
            break;
        case LOOK_IN_MOTHERSHIP:
            if (newDriveCmd){
                newDriveCmd = false;
                stopMotors();
                claw.Gripper_Open();
                claw.Servo_SetAngle(58);
                cameraServo.writeMicroseconds(DEG_TO_US(45));
                distDrive();
            }
            break;
        default:
            break;
        }
  rosUpdate(false); //done with commands, ack back to pi
}

/*
   * Drives the set distance (stored in float distance). Returns when distance setpoint is achieved.
   * Interruptable at least every SAMPLE_PERIOD by rosUpdate(), which calls nh.spinOnce().
   */
  bool hasMoved(volatile long errors[2]) {
      static long lastMovedTime = millis();
      static long last_errors[2] = {0,0};
      
      bool moved = (errors[0] != last_errors[0] || errors[1] != last_errors[1]);
      
      if (moved){
          lastMovedTime = millis();  
      }
      last_errors[0] = errors[0];
      last_errors[1] = errors[1];
      if (millis() - lastMovedTime > MOVE_TIMEOUT){
          lastMovedTime = millis();
          return false;
      }
      return true;
  }
   

/*
 * Drives the set distance (stored in float distance). Returns when distance setpoint is achieved.
 * Interruptable at least every SAMPLE_PERIOD by rosUpdate(), which calls nh.spinOnce().
 */
void distDrive(){
    resetEncoderCounts();
    float posError[NUM_MOTORS] = {0,0};
    float maximumSpeed = MAX_SPEED;
    if (distanceSetpoint < 0){
        maximumSpeed = MAX_REVERSE_SPEED;
    }
    bool moving = true;
    do{
        do{
            for (int i = 0; i < NUM_MOTORS; i++){
                // Find error in inches
                posError[i] = distanceSetpoint - ((encCounts[i]*WHEEL_CIRC)/COUNTS_PER_REV);
                // Convert position error to a velocity setpoint for each wheel
                maximumSpeed = turboAvailible ? MAX_SPEED + TURBO : MAX_SPEED;
                velocitySetpoints[i] = posErrorToAngVel(posError, i, maximumSpeed);
                moving = hasMoved(encCounts);
            }
            velDrive();
            rosUpdate(true);
        } while (!isZero(posError, false) && moving);
        stopMotors();
        delay(OVERSHOOT_DELAY_MS);
        for (int i = 0; i < NUM_MOTORS; i++){
            // Find error in inches
            posError[i] = distanceSetpoint - ((encCounts[i]*WHEEL_CIRC)/COUNTS_PER_REV);
        }
    } while (!isZero(posError, false) && moving);
    float endAngle = getHeading();
    stopMotors();
    turboAvailible = false;
    float avgError = 0;
    for (int i = 0; i < NUM_MOTORS; i++){
        avgError += posError[i];
    }
    avgError = avgError/NUM_MOTORS;
    robot_pose.x += (distanceSetpoint-avgError)*cos(DEG2RAD*endAngle);
    robot_pose.y += (distanceSetpoint-avgError)*sin(DEG2RAD*endAngle);
    robot_pose.theta = getHeading();
}


/*
 * Implements proportional control for position. 
 * Takes in the array of errors (inches) and the motor number and returns the angular velocity setpoint (float, rad/s).
 * Also implements feedback between the two wheels to keep the robot on a straight path.
 */
float posErrorToAngVel(float* errors, int motor, float maximumSpeed){
    float angVel = (errors[motor]*K_P_DIST);
    //saturate vel values
    if (abs(angVel) > maximumSpeed) angVel =  angVel > 0 ? maximumSpeed : -maximumSpeed;
    else if (abs(angVel) < MIN_SPEED) angVel =  angVel > 0 ? MIN_SPEED : -MIN_SPEED;
    
    // Do error correction to keep wheels consistent with each other
    int otherMotor = (motor == L) ? R : L;
    float motorDiff = errors[motor] - errors[otherMotor];
    float motorDiffCorrection = K_DIFF * motorDiff;
    if (motorDiffCorrection > MAX_CORRECTION) motorDiffCorrection = MAX_CORRECTION;
    if (motorDiffCorrection < -MAX_CORRECTION) motorDiffCorrection = -MAX_CORRECTION;

    angVel += motorDiffCorrection;
    return angVel;
}

/*
 * Turns the robot the angle stored in angleSetpoint. Returns when the angle is achieved. 
 */
void turn(){
//resetEncoderCounts();
    //CCW rotation, left is reverse, right is forward
    //angleSetpoints[L] = -angleSetpoint - TURN_ESS_GAIN*angleSetpoint;
    //angleSetpoints[R] = angleSetpoint + TURN_ESS_GAIN*angleSetpoint;
    turning = true;
    double angError = 0;
    float targetAngle = boundAngle(getHeading() + angleSetpoint);
    //reset integral
    for (int i = 0; i < NUM_MOTORS; i++){
        turnIntegral[i] = 0;
    }
    do {
        do {
            // Find error in degrees
            double angle = getHeading();
            angError = boundAngle(targetAngle - angle);
            //Serial.println("Error: " + String(angError));
            // Convert from error in degrees to an angVel setpoint for each wheel 
            for (int i = 0; i < NUM_MOTORS; i++){
                velocitySetpoints[i] = angErrorToAngVel(angError, i);
            }
        //changes motor output
        velDrive(); //enforces set sample period
        rosUpdate(true  );
        } while (abs(angError) > 1);
        stopMotors();
        // Delay for motor overshoot calculation
        delay(OVERSHOOT_DELAY_MS);
        //recalc error for overshoot, reset integrator
        for (int i = 0; i < NUM_MOTORS; i++){
            turnIntegral[i] = 0;
            double angle = getHeading();
            angError = boundAngle(targetAngle - angle);
        }
    } while (abs(angError) > 1);
    stopMotors();
    robot_pose.theta = getHeading();
    turning = false;
}

/*
 * Takes in the angle error in degrees (float) and applies a gain to return the ang vel setpoint (float).
 * The proportional control gain is set by K_P_ANG.
 */
float angErrorToAngVel(float error, int motor){
    turnIntegral[motor] += SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * error;
    float angVel = (error*K_P_ANG + turnIntegral[motor]*K_I_ANG);
    if (motor == L) angVel = -angVel;
    if (abs(angVel) > MAX_TURN_SPEED) {
        return angVel > 0 ? MAX_TURN_SPEED : -MAX_TURN_SPEED;
    }
    else if (abs(angVel) < MIN_SPEED) {
        return angVel > 0 ? MIN_SPEED : -MIN_SPEED;
    }
    else return angVel;
} 


/*
   void velDrive()
   Takes in a velocity setpoint and drives forward at that speed.
*/
void velDrive() {
    unsigned long timeRef = millis();

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
    if (angVelSetpoint == 0) return 0;
    if (velocities[motor] > (MAX_SPEED - TURBO_SETPOINT)) turboAvailible = true;

    float error = angVelSetpoint - velocities[motor];

    integralControlValue[motor] += SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * error;
    float kp = turning ? K_P_TURN : K_P;
    float ki = turning ? 0 : K_I;
    int PWMSetpoint = (int)(error * kp + integralControlValue[motor] * ki);
    
    //saturate PWM value
    if (angVelSetpoint == 0) return 0;
    if (abs(PWMSetpoint) > 255) {
        return PWMSetpoint > 0 ? 255 : -255;
    }
    else return PWMSetpoint;
}




/*
 * Updates the position of the robot based on the velocity values of the wheels. The function
 * uses an approximation of turning/driving straight for the case when the robot is driving in an 
 * arc. 
 */
void updatePosition(){
    volatile static double previousTheta = getHeading();
    robot_pose.theta = getHeading();
    
    float leftDelta = deltaPosition[L]*INCHES_PER_COUNT;
    float rightDelta = deltaPosition[R]*INCHES_PER_COUNT;
    double dTheta = boundAngle(robot_pose.theta - previousTheta);
    if (fabs(velocitySetpoints[L] + velocitySetpoints[R]) < 0.1){}
    else if(fabs(dTheta) <  STRAIGHT_THRESH){
        // Robot is going straight, update x and y based on the current heading
        robot_pose.x += leftDelta*cos(DEG2RAD*robot_pose.theta);
        robot_pose.y += rightDelta*sin(DEG2RAD*robot_pose.theta);
        
    }
    else {
        // Robot is driving at an arc, approximate position as initial turn then straight path
        float arcLength = (leftDelta + rightDelta)/2;
        robot_pose.x += (arcLength*cos(DEG2RAD*robot_pose.theta));
        robot_pose.y += (arcLength*sin(DEG2RAD*robot_pose.theta));
    }
    previousTheta = robot_pose.theta;
}


/*
 * Takes in motor (m) and PWM Value (pwm) and sets each motor and direction.
 */
void setMotor(int m, int pwm) {
    //negative on left motor is forwards
    pwm = (m == L) ? -pwm : pwm;
  
    //get motor pins
    int M_pin = (m == L) ? ML : MR;
    int E_pin = (m == L) ? EL : ER;
  
    //set direction and speed
    digitalWrite(M_pin, pwm > 0 ? HIGH : LOW);
    analogWrite (E_pin, abs(pwm));
}

void do_stop() {
    //nh.loginfo("STOPPING*************");
    velocitySetpoints[L] = 0;
    velocitySetpoints[R] = 0;
    distanceSetpoint = 0;
    angleSetpoint = 0;
    stopMotors();
}

/*
 * Sets all motors to zero speed. Also resets integral control values to 0.
 */
  void stopMotors() {
    for (int i = 0; i < NUM_MOTORS; i++){
        setMotor(i, 0);
        integralControlValue[i] = 0;
    }
}

/*
 * Resets encoder counts and last counts to zero.
 */
void resetEncoderCounts() {
    for (int i = 0; i < NUM_MOTORS; i++){
        encCounts[i] = 0;
        lastCounts[i] = 0;
    }
}

/*
 * Helper function that takes in an array and checks it against the ZERO_ERROR_MARGIN constant. 
 * The function returns true if the values in the array are less than the
 * error margin. Distinguishes between zero error margin for turning (TURN_ZERO_ERROR_MARGIN) and going straight. 
 */
bool isZero(float* errors, bool turning){                                                                                                               
    for (int i = 0; i < NUM_MOTORS; i++){ 
        float margin = turning ? TURN_ZERO_ERROR_MARGIN : ZERO_ERROR_MARGIN;                                               
        if (abs(errors[i]) > margin) return false;          
    }                                                                           
    return true;                                                                
} 

/*
 * Returns the absolute angular position of the robot in degrees bounded -180 to 180.
 */
double getHeading(){
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    double angle = (360 - orientationData.orientation.x) + 90;
    angle = boundAngle(angle);
    return angle;
}

/*
 * Helper function that takes in an angle (double) and returns the angle bounded -180 to 180 degrees.
 */
double boundAngle(double angle){
    while(fabs(angle) > 180){
        angle += angle > 0 ? -360 : 360;
    }
    return angle;
}

/*
 * Takes in the motor number and updates the robot's velocity based on angPosition/SAMPLE_PERIOD.
 */
void updateVelocity(int motor) {
    long newEncCounts = encCounts[motor];
    deltaPosition[motor] = newEncCounts - lastCounts[motor];
    lastCounts[motor] = newEncCounts;
    velocities[motor] = (deltaPosition[motor] * 2 * PI) / (SAMPLE_PERIOD * SECONDS_PER_MILLISECOND * COUNTS_PER_REV);
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
