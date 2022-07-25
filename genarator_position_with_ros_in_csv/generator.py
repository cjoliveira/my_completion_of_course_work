#!/usr/bin/python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import csv


def get_original_position(req):
    global pos_aux
    global original_pos
    if pos_aux==0:
        pos_aux = 1
        original_pos=req

def scan(req):
    global aux
    global scan_readings
    if aux == 0:
        aux = 1
        Scan_Data = req
        Laser_Ang_max = Scan_Data.angle_max
        Laser_Ang_min = Scan_Data.angle_min
        Laser_Ang_Incre = Scan_Data.angle_increment
        Laser_Ang_ranges = Scan_Data.ranges
        Angulos = np.arange(Laser_Ang_min,Laser_Ang_max,Laser_Ang_Incre)
        scan_readings = np.array(Laser_Ang_ranges)
        # for i in range(len(Laser_Ang_ranges)):
        #     print('A leitura do Angulo ', np.rad2deg(Angulos[i]), 'é:',Laser_Ang_ranges[i])

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


if __name__ == "__main__":
    global aux
    global scan_readings
    global pos_aux
    global original_pos

    aux = 0
    scan_readings = []
    pos_aux = 0
    original_pos = Odometry()
    rospy.init_node('p3dx_localization', anonymous=False)
    rate = rospy.Rate(10)
    Localization = rospy.Subscriber("/p3dx/laser/scan", LaserScan, scan) 
    rospy.sleep(1)

    Position = rospy.Subscriber("/p3dx/base_pose_ground_truth", Odometry, get_original_position)
    rospy.sleep(1)
    Pos_Original = [0,0,0]
    Pos_Original[0] = original_pos.pose.pose.position.x
    Pos_Original[1] = original_pos.pose.pose.position.y
    roll, pitch, yam = quaternion_to_euler(original_pos.pose.pose.orientation.x, original_pos.pose.pose.orientation.y, original_pos.pose.pose.orientation.z, original_pos.pose.pose.orientation.w)
    Pos_Original[2] = math.degrees(yam)

    print("#################### Original position #####################")
    print("x=", Pos_Original[0], " y=", Pos_Original[1], " theta:", Pos_Original[2])

    l = np.array(scan_readings)
    f = open('bd_ros.csv', 'w', newline='', encoding='utf-8')
    w = csv.writer(f)
    w.writerow([Pos_Original[0], Pos_Original[1], Pos_Original[2],scan_readings[0],scan_readings[1],scan_readings[2],scan_readings[3], scan_readings[4],
    scan_readings[5], scan_readings[6], scan_readings[7], scan_readings[8], scan_readings[9]])
    f.close()




