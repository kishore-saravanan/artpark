#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import ObjectCount
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

import time
vels = Twist()
picker_run = Float32()
flag = 0

def move(box):
    global flag
    id1 = box.bounding_boxes[0].id
    x_min = box.bounding_boxes[0].xmin
    x_max = box.bounding_boxes[0].xmax
    center = x_min+((x_max-x_min)/2)
    print(center)
    if center < 600:
        print("Right")
        vels.linear.x = 0.0
        vels.angular.z = 0.5
        pub.publish(vels)
    elif center > 680:
        print("Left")
        vels.linear.x = 0.0
        vels.angular.z = -0.5
        pub.publish(vels)
    elif id1:
        print("Forward")
        vels.linear.x = 0.2
        vels.angular.z = 0.0
        pub.publish(vels)
        flag = 1
        
    
    
    


def detect(obj):
    global flag
    loop_rate = rospy.Rate(1)
    print("Objects detected: ",obj.count)
    if not obj.count:
        if flag:
            vels.linear.x = 0.0
            vels.angular.z = 0.0
            pub.publish(vels)
            picker_run.data = 1.0
            picker_pub.publish(picker_run)
            print("Picking Trash")
            rospy.sleep(10)
            picker_run.data = 0.0
            picker_pub.publish(picker_run)
            flag = 0
        vels.linear.x = 0.0
        vels.angular.z = -0.5
        pub.publish(vels)
        '''vels.linear.x = 0.0
        vels.angular.z = 0.0
        pub.publish(vels)'''

    else:
        sub_box = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, move)
    '''loop_rate = rospy.Rate(10)
    vels.linear.x = 0.4
    vels.angular.z =0
    
    if(s.ranges[0] <0.6 or s.ranges[1] <0.6 or  s.ranges[2] <0.6 or s.ranges[3] <0.6 or s.ranges[4] <0.6 or s.ranges[5] <0.6 or s.ranges[-5] <0.6 or s.ranges[-4] <0.6 or s.ranges[-3] <0.6 or s.ranges[-2] <0.6 or s.ranges[-1] <0.6):
        vels.linear.x =0
        pub.publish(vels)
    if(vels.linear.x ==0):
        rotate(s)
        pub.publish(vels)

    loop_rate.sleep()
    pub.publish(vels)'''

def rotate():
    print("Rotate")
    vels.linear.x = 0.0
    vels.angular.z = 0.5
    pub.publish(vels)


'''def rotate(b):
    loop_rate = rospy.Rate(10)
    if (b.ranges[0] <3 or b.ranges[1] <3 or  b.ranges[2] <3 or b.ranges[3] <3 or b.ranges[4] <3 or b.ranges[5] <3):
        vels.angular.z=1
        #pub.publish(vels)
    elif (b.ranges[-5] <3 or b.ranges[-4] <3 or b.ranges[-3] <3 or b.ranges[-2] <3 or b.ranges[-1] <3):   
        vels.angular.z=-1
        #pub.publish(vels)
    else:
        pub.publish(vels)

    pub.publish(vels)
    loop_rate.sleep()'''

if __name__ == "__main__":
    rospy.init_node('scan_node12', anonymous=True)
    pub = rospy.Publisher('/cmd_vel' , Twist , queue_size=100)
    picker_pub = rospy.Publisher('/pickerwheel_run', Float32, queue_size=10)
    rotate()
    sub_object = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, detect)
    time.sleep(2)
    rospy.spin()