#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from getkey import getkey, keys
from turtlesim.msg import Pose

class MyTurtle:
    
    def __init__(self):
        rospy.init_node('robot_cleaner', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, self.update_pose)

        self.pose = Pose()

    def start(self):

        pose_list = []
        vel_msg = Twist()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        speed = 1

        while not rospy.is_shutdown():
            key = getkey()
            current_distance = 0

            if key == keys.RIGHT:
                t0 = rospy.Time.now().to_sec()
                vel_msg.linear.x = 1
                while(current_distance < 1):
                    self.velocity_publisher.publish(vel_msg)
                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)
                vel_msg.linear.x = 0
                self.velocity_publisher.publish(vel_msg)
                pose_list.append([self.pose.x, self.pose.y])

            if key == keys.LEFT:
                t0 = rospy.Time.now().to_sec()
                vel_msg.linear.x = -1
                while(current_distance < 1):
                    self.velocity_publisher.publish(vel_msg)
                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)
                vel_msg.linear.x = 0
                self.velocity_publisher.publish(vel_msg)
                pose_list.append([self.pose.x, self.pose.y])

            if key == keys.UP:
                t0 = rospy.Time.now().to_sec()
                vel_msg.linear.y = 1
                while(current_distance < 1):
                    self.velocity_publisher.publish(vel_msg)
                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)
                vel_msg.linear.y = 0
                self.velocity_publisher.publish(vel_msg)
                pose_list.append([self.pose.x, self.pose.y])

            if key == keys.DOWN:
                t0 = rospy.Time.now().to_sec()
                vel_msg.linear.y = -1
                while(current_distance < 1):
                    self.velocity_publisher.publish(vel_msg)
                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)
                vel_msg.linear.y = 0
                self.velocity_publisher.publish(vel_msg)
                pose_list.append([self.pose.x, self.pose.y])
            
            if key == 'q':
                print(pose_list)
                break
            
            current_distance = 0
    
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
if __name__ == '__main__':
    try:
        turtle = MyTurtle()
        turtle.start()
    except rospy.ROSInterruptException:
        pass