
#include <AccelStepper.h>
#include <Wire.h>

const int addr = 0x43;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 8, 9); //step, dir

enum COLOR{RED, GREEN, BLUE, GRAY, CYAN, MAGENTA, YELLOW};
int pos[7] = {0,460,920,1380,1840,2300,2760};
int storeOffset = -1610;
bool isDropping = false;
enum CMD{GET_COLOR, DUMP, KILL_ALL};

int dump_pin = 2;
int force_stop = 6;
int conveyor_start = 7;

//conveyor
int enA = 3;
int in1 = 4;
int in2 = 5;

//solenoid
int enB = 10;
int in3 = 11;
int in4 = 12;


void setup()
{ 
  Serial.begin(9600);
  Wire.begin(addr);
  Wire.onReceive(getColor);
  Wire.onRequest(nothing);
  
  stepper.setMaxSpeed(50000);
  stepper.setAcceleration(30000);

  // set all the motor control pins to outputs
    pinMode(dump_pin, INPUT_PULLUP);
    pinMode(conveyor_start, INPUT_PULLUP);
    pinMode(force_stop, INPUT_PULLUP);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
  
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite (enA, 0);
  
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite (enB, 0);

    setConveyor(false);
}

void loop()
{

  if (Serial.available() > 0){
    int goTo = Serial.read() - 48;
    Serial.println(goTo);
    if (goTo == 9) isDropping = !isDropping;
    else stepper.moveTo(pos[goTo] + (isDropping ? 0:storeOffset));
  }
  stepper.run();

   if (digitalRead(dump_pin) == LOW){
        delay(50);
        if (digitalRead(dump_pin) == LOW){
            isDropping = true;
            stepper.moveTo(RED + (isDropping?0:storeOffset) );
            openHatch();
        }
    }

    if (digitalRead(force_stop) == LOW){
        delay(50);
        if (digitalRead(force_stop) == LOW){
            kill_all();
        }
    }
    
    if (digitalRead(conveyor_start) == LOW){
        delay(50);
        if (digitalRead(conveyor_start) == LOW){    
            stepper.moveTo(pos[0] + storeOffset);
            setConveyor(true);
        }
    }

 }

void setConveyor(bool on){
    if (on){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite (enA, 200);          
    } 
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite (enA, 0);
    } 
}

void openHatch(){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite (enB, 255);
    delay(500);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite (enB, 0);
  
}

void kill_all(){
    isDropping = false;
    stepper.moveTo(RED + (isDropping?0:storeOffset) );
    setConveyor(false);
  
}

void getColor(int num_bytes) {
    // The smbus implementation sends over a command and the number of bytes it has in its payload
    // before it actually, dir sends over the real data
    int cmd = Wire.read();
    int bytes = Wire.read();
    
    // Do a sanity check to make sure that the data received follows the correct format
    if (num_bytes == bytes + 2) {
        if (cmd == GET_COLOR) {
          int color = i2cGetInt();
          stepper.moveTo(pos[color] + isDropping?0:storeOffset);
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

void nothing(){}

// Reads the next 2 bytes from the i2c bus and splices them together to make a signed 16-bit integer.
int i2cGetInt() {
    return ((Wire.read() << 8) | Wire.read());
}

void dumpData() {
    while (Wire.available()) Wire.read();
}

