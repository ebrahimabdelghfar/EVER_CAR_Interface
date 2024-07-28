#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String , Float32

steering = Float32()
velocity = Float32()
command = String()

rospy.init_node("car_commands")

def steering_callback(msg:Float32):
    steering.data = msg.data
    pass

def velocity_callback(msg:Float32):
    velocity.data = msg.data
    pass

pub = rospy.Publisher("/publish/primitive",String,queue_size=1)
rospy.Subscriber("/in_Car_velocity_in_KM/H" , Float32 , callback = velocity_callback )
rospy.Subscriber("/in_Car_steering_in_degree" , Float32 , callback = steering_callback )
rate = rospy.Rate(5)

if __name__ == "__main__" :
    while not rospy.is_shutdown():
        command.data = str(velocity.data)+" "+str(steering.data)
        pub.publish(command)
        rate.sleep()
        pass
    pass