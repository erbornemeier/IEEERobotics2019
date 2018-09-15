
#include <AccelStepper.h>
#include <Wire.h>

const int addr = 0x43;

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


// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 8, 9); //dir, step

enum COLOR{RED, GREEN, BLUE, GRAY, CYAN, MAGENTA, YELLOW};
int pos[7] = {0,460,920,1370,1840,2290,2750};
enum CMD{GET_COLOR, DUMP, KILL_ALL};
int dump_offset = 1375;

bool is_dumping = false;

void setup()
{ 
    Serial.begin(9600);
    Wire.begin(addr);
    Wire.onReceive(getColor);
    Wire.onRequest(nothing);
    
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    
    stepper.setMaxSpeed(500000);
    stepper.setAcceleration(3000000);
  
    // set all the motor control pins to outputs
    pinMode(dump_pin, INPUT_PULLUP);
    pinMode(conveyor_start, INPUT_PULLUP);
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

    
    stepper.moveTo(RED + (is_dumping?0:dump_offset) );
  
}

void loop()
{
stepper.run();
    
    Serial.println(stepper.distanceToGo());

    if (Serial.available() > 0){
        int goTo = Serial.read() - 48;
        stepper.moveTo(pos[goTo]);
    }
    
    

    if (digitalRead(dump_pin) == LOW){
        delay(50);
        if (digitalRead(dump_pin) == LOW){
            is_dumping = true;
            stepper.moveTo(RED + (is_dumping?0:dump_offset) );
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
            
            setConveyor(true);
        }
    }

    /*
    delay(5000);
    openHatch();
    delay(1000);
    setConveyor(false);
    while(true){}
    */
  
 }


void setConveyor(bool on){
    if (on){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite (enA, 150);          
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
    is_dumping = false;
    stepper.moveTo(RED + (is_dumping?0:dump_offset) );
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
            stepper.moveTo(color + is_dumping?0:dump_offset);
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


