#include <Colorduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include "display_letters.h"

/*****************************************
* ROS
*****************************************/
ros::NodeHandle nh;

void letterCallback(const std_msgs::Int32& letter) {
  int data = letter.data;
  nh.loginfo(String(data).c_str());
}

ros::Subscriber<std_msgs::Int32> sub("letter_identifier", &letterCallback);

//int8_t m_shape[][2] = {
//                    {1,1},
//                    {1,2},
//                    {1,3},
//                    {1,4},
//                    {1,5},
//                    {1,6},
//                    {2,5},
//                    {3,4},
//                    {4,4},
//                    {5,5},
//                    {6,6},
//                    {6,5},
//                    {6,4},
//                    {6,3},
//                    {6,2},
//                    {6,1}
//                      };

void setup()
{
    Colorduino.Init();
    // compensate for relative intensity differences in R/G/B brightness
    // array of 6-bit base values for RGB (0~63)
    // whiteBalVal[0]=red
    // whiteBalVal[1]=green
    // whiteBalVal[2]=blue
    unsigned char whiteBalVal[3] = {40,63,50}; // for LEDSEE 6x6cm round matrix
    Colorduino.SetWhiteBal(whiteBalVal);

//    for (int i = 0; i < 10; i++) loading_screen();

    Colorduino.ColorFill(0,0,0);

  nh.initNode();
  nh.subscribe(sub);

//  while(!nh.connected()) {
//    nh.spinOnce();
//  }
}


void loop (){
  for (int i = A; i <= F; i++) {
      displayLetter(i);
      delay(750);
  }
  nh.spinOnce();
 
}

//void loading_screen(){
//  
//    //blue on
//    for (int i = 0; i < 16; i++){
//        Colorduino.FlipPage();
//        int x = m_shape[15-i][0];
//        int y = m_shape[15-i][1];
//        Colorduino.SetPixel(x,y, 0x21, 0x31, 0x4d);
//        Colorduino.FlipPage();
//        delay(50);
//        
//    }
//
//    //flash
//    for (int i = 0; i < 3; i++){
//        Colorduino.FlipPage();
//        delay(250);
//        Colorduino.FlipPage();
//        delay(500);
//    }
//    
//    //blue off
//    for (int i = 0; i < 16; i++){
//        Colorduino.FlipPage();
//        int x = m_shape[i][0];
//        int y = m_shape[i][1];
//        Colorduino.SetPixel(x,y, 0x0, 0x0, 0x0);
//        Colorduino.FlipPage();
//        delay(50);
//        
//    }
    
//}
