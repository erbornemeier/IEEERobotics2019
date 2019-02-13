#include <Colorduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose2D.h>
#include "display.h"

/*****************************************
* ROS
*****************************************/
ros::NodeHandle_<ArduinoHardware, 2, 0, 80, 105> nh;

void letterCallback(const std_msgs::UInt8& data) {
    uint8_t letter = data.data;
    nh.loginfo(String(letter+'A').c_str());
    if (letter == 0xFF) clear_screen();
    else displayLetter(letter);
}

void positionCallback(const geometry_msgs::Pose2D& data) {
    showRobotPos(round(data.x-0.5),round(data.y-0.5));
}

ros::Subscriber<std_msgs::UInt8> li("display_letter", &letterCallback);
ros::Subscriber<geometry_msgs::Pose2D> rp_led("display_robot_pos", &positionCallback);
  
void setup()
{
    Colorduino.Init();
    unsigned char whiteBalVal[3] = {40,63,50}; 
    Colorduino.SetWhiteBal(whiteBalVal);

    nh.initNode();
    nh.subscribe(li);
    nh.subscribe(rp_led);

    while(!nh.connected()) {
        loading_screen();
        nh.spinOnce();
    }
    Colorduino.FlipPage();
    Colorduino.ColorFill(0,0x60,0);

}


void loop (){
    nh.spinOnce();
    delay(1);
}
