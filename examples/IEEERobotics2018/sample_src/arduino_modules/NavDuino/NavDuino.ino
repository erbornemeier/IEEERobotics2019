/* 
    Navigation Arduino

    Module that contains the control loop for moving the robot chassis around
    the playing field. Uses I2C to communicate with the navigatin Pi in order
    to receive location instructions
*/

#include <Wire.h>

#include <stdio.h>
#define MAX_BUF 32

const int I2C_ADDR = 0x42;

const int GET_TARG_POS = 0;
const int GET_CUR_POS = 1;

enum POS_IDX {X, Y, R, NUM_AXIS};

volatile int target[NUM_AXIS];
volatile int dir;

volatile int pos[NUM_AXIS];

volatile int completed;

// TODO: Remove debug code
int l_target[NUM_AXIS];
int l_dir;

int l_pos[NUM_AXIS];

void setup() {
    Serial.begin(9600);

    Wire.begin(I2C_ADDR);
    Wire.onReceive(getPosition);
    Wire.onRequest(checkDone);

    completed = 0;
}

void loop() {
    char buf[MAX_BUF];
    if (target[X] != l_target[X] || target[Y] != l_target[Y] 
        || target[R] != l_target[R] || dir != l_dir) {
        sprintf(buf, "Target: (%d, %d, %d) %d", target[X], target[Y], target[R], dir);
        Serial.println(buf);
        for (int i = 0; i < 3; i++)
            l_target[i] = target[i];
        l_dir = dir;
    }
    else if (pos[X] != l_pos[X] || pos[Y] != l_pos[Y] || pos[R] != l_pos[R] 
             || dir != l_dir) {
        sprintf(buf, "Pos: (%d, %d, %d)", pos[X], pos[Y], pos[R]);
        Serial.println(buf);
        for (int i = 0; i < 3; i++)
            l_pos[i] = pos[i];
    }
}

void getPosition(int num_bytes) {
    // The smbus implementation sends over a command and the number of bytes it has in its payload
    // before it actually, dir sends over the real data
    int cmd = Wire.read();
    int bytes = Wire.read();
    
    // Do a sanity check to make sure that the data received follows the correct format
    if (num_bytes == bytes + 2) {
        if (cmd == GET_TARG_POS) {
            for(int i = 0; i < 3; i++)
                target[i] = i2cGetInt();
            dir = i2cGetInt();
        }
        else if (cmd == GET_CUR_POS) {
            for (int i = 0; i < 3; i++)
                pos[i] = i2cGetInt();
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

void checkDone() {
    Wire.write(completed);
    completed = completed == 0 ? 1 : 0;
}

// Reads the next 2 bytes from the i2c bus and splices them together to make a signed 16-bit integer.
int i2cGetInt() {
    return ((Wire.read() << 8) | Wire.read());
}

void dumpData() {
    while (Wire.available()) Wire.read();
}

