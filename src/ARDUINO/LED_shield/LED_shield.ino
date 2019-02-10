#include <Colorduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include "display_letters.h"

/*****************************************
* ROS
*****************************************/
ros::NodeHandle_<ArduinoHardware, 1, 0, 80, 105> nh;

void letterCallback(const std_msgs::Int32& data) {
    int letter = data.data;
    nh.loginfo(String(letter).c_str());
    displayLetter(letter);
}

ros::Subscriber<std_msgs::Int32> sub("letter_identifier", &letterCallback);

void setup()
{
    Serial.begin(57600);
    Colorduino.Init();
    unsigned char whiteBalVal[3] = {40,63,50}; 
    Colorduino.SetWhiteBal(whiteBalVal);

    nh.initNode();
    nh.subscribe(sub);

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
