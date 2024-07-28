#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String , Float32

rospy.init_node("Car_interface")

steering = Float32()
velocity = Float32()

def callback(msg:String):
    global steering , velocity
    recieved_string = msg.data
    parts = recieved_string.split()
    velocity.data = float(parts[0])
    steering.data = float(parts[1])
    CAR_VELOCITY.publish(velocity)
    CAR_STEERING.publish(steering)
    pass

rospy.Subscriber("subscribe",String,callback=callback)
CAR_VELOCITY = rospy.Publisher("/sensor_Car_velocity_in_KM/H" , Float32 , queue_size = 1)
CAR_STEERING = rospy.Publisher("/sensor_Car_steering_in_degree" , Float32 , queue_size= 1)

if __name__ == "__main__":
    rospy.spin()
