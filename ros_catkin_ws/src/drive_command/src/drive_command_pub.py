#!/usr/bin/env python                                                           
                                                                                
import rospy                                                                    
from std_msgs.msg import Float32
                                                                                
def publisher():                                                                
    pub = rospy.Publisher("drive_command", Float32, queue_size=1)                  
    rospy.init_node("drive_publisher", anonymous=True)                         
                                                                                
    while not rospy.is_shutdown():                                              
        drive_command = float(raw_input("Enter : drive (in) >"))
        msg = Float32()
        msg.data = drive_command
        pub.publish(msg) 
                                                                                
def main():                                                                     
    try:                                                                        
        publisher()                                                             
    except rospy.ROSInterruptException:                                         
        pass                                                                    
                                                                                
if __name__ == '__main__':                                                      
    main()           
