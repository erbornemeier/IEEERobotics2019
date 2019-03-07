#include <Colorduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose2D.h>
#include "display.h"

volatile uint8_t block_positions[6] = {0};
volatile uint8_t num_blocks_recieved = 0;

/*****************************************
* ROS
*****************************************/
ros::NodeHandle_<ArduinoHardware, 3, 0, 80, 105> nh;

void letterCallback(const std_msgs::UInt8& data) {
    uint8_t letter = data.data;
    //nh.loginfo(String(letter+'A').c_str());
    if (letter == 0xFF) clear_screen();
    else displayLetter(letter);
}

void blockCallback(const std_msgs::UInt8& data) {
    uint8_t block_pos = data.data;
    if (block_pos == 0xFF) 
        clear_screen();
    else if (block_pos != 0xFE)
        block_positions[num_blocks_recieved++] = block_pos;
    displayBlocks(block_positions, num_blocks_recieved);
}

void positionCallback(const geometry_msgs::Pose2D& data) {
    showRobotPos(round(data.x-0.5),round(data.y-0.5));
}

ros::Subscriber<std_msgs::UInt8> li("display_letter", &letterCallback);
ros::Subscriber<std_msgs::UInt8> db("display_block", &blockCallback);
ros::Subscriber<geometry_msgs::Pose2D> rp_led("display_robot_pos", &positionCallback);
  
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

    while(!nh.connected()){
        loading_screen();
        nh.spinOnce();
    }
    displayBlocks(block_positions, num_blocks_recieved);
}


void loop (){
    nh.spinOnce();
    delay(1);
}
