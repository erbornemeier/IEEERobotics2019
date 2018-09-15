/*
	MOTOR POSITION PID CONTROLLER

	Authors: Easton Bornemeier & Daichi Jameson

	Program to drive and control the motors using the I2C connection from the Raspberry Pi to track destination
	position and current position

*/

long Error[10]; //difference between actual values and intended values with back-buffer storage of 10
long I_Accum; //accumulation of error over time for use on the integral term
long PID; //actual PID value calculated

int P; //proportional term
int I; //integral term
int D; //derivative term

byte Divider; //shift term to bit shift the large PID value to a more feasible range

int currentPos; //current position from the Pi
int destPos;	//destination position from the Pi

byte M_VALS[4]; //sign values for the motor controller
byte E_VALS[4]; //speed values for the motor controller

//Motor pin values in order FL, FR, BL, BR
int M_PINS[4] = {2, 4, 7, 8};
int E_PINS[4] = {3, 5, 6, 9};


void setup(){

	for (byte i = 0; i < 4, i++){
		pinMode(M_PINS[i], OUTPUT);
		pinMode(E_PINS[i], OUTPUT);
	}

}

void loop(){


}

void getError(){

	//shift error values back
	for (byte i = 9; i > 0; i--){
		Error[i] = Error[i-1];
	}

	//load in new error value
	Error[0] = destPos - currentPos; //TODO: LOAD THIS IN FROM I2C

}

void calcPID(){
	//control values
	P = 2000;
	I = 25;
	D = 0;
	Divider = 10;

	//proportional component
	PID = Error[0]*P;

	//integral component
	I_Accum += Error[0];
	PID += I*I_Accum;

	//derivative component
	long deltaError = Error[0] - Error[9];
	PID += D*deltaError;

	//shift
	PID>>Divider;

	//check bounds
	if (PID >=  255) PID =  255;
	if (PID <= -255) PID = -255;

	//TODO: FIGURE OUT HOW TO WRITE TO THE MOTORS IN THIS WAY, ARE THEY ALL THE SAME VALUE ON FORWARD MOVEMENT?
}

void writeToMotors(){

	for (byte i = 0; i < 4; i++){
		digitalWrite(M_PINS[i], M_VALS[i]);
		analogWrite(E_PINS[i], E_VALS[i]);
	}

}