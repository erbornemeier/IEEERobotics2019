#include <Colorduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose2D.h>
#include "display.h"

#define NORMAL 0
#define LETTER 1
#define WAITING 2
volatile uint8_t display_state = WAITING;

volatile uint8_t block_positions[6] = {0};
volatile uint8_t num_blocks_recieved = 0;
uint8_t robot_x = 4, robot_y = 4, letter = 0;

/*****************************************
* ROS
*****************************************/
ros::NodeHandle_<ArduinoHardware, 4, 0, 80, 105> nh;

void letterCallback(const std_msgs::UInt8& data) {
    letter = data.data;
}

void blockCallback(const std_msgs::UInt8& data) {
    block_positions[num_blocks_recieved++] = data.data;
}

void positionCallback(const geometry_msgs::Pose2D& data) {
    robot_y = round(data.x/12.0-0.5);
    robot_x = round(data.y/12.0-0.5);
}

void changeDisplayState(const std_msgs::UInt8& data){
    display_state = data.data;  
}

ros::Subscriber<std_msgs::UInt8> li("display_letter", &letterCallback);
ros::Subscriber<std_msgs::UInt8> db("display_block", &blockCallback);
ros::Subscriber<geometry_msgs::Pose2D> rp_led("robot_pose", &positionCallback);
ros::Subscriber<std_msgs::UInt8> cds("change_display_state", &changeDisplayState);

  
void setup()
{
    Colorduino.Init();
    unsigned char whiteBalVal[3] = {40,63,50}; 
    Colorduino.SetWhiteBal(whiteBalVal);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(li);
    nh.subscribe(db);
    nh.subscribe(rp_led);
    nh.subscribe(cds);

    while(!nh.connected()){
        loading_screen();
        nh.spinOnce();
    }
}


void loop (){
    Colorduino.FlipPage();
    switch(display_state){
        case NORMAL:
            Colorduino.ColorFill(0x50, 0x08, 0x00); //martian color
            displayBlocks(block_positions, num_blocks_recieved);
            displayRobotPos(robot_x, robot_y);
            break;
        case LETTER:
            Colorduino.ColorFill(0x0, 0x0, 0x0);
            displayLetter(letter);
            break;
        case WAITING:
            Colorduino.ColorFill(0x0, 0x20, 0x00);
            break;
        default:
            break;
    }
    Colorduino.FlipPage();
    nh.spinOnce();
    delay(100);
}
