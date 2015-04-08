#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from ros_myo.msg import EmgArray
from geometry_msgs.msg import Twist
import numpy as np

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


VEL_TOPIC = "/key_vel"

class DriveMyo():
    """
    """
    def __init__(self):
        self.new_data = False
        self.curr_vol = 0.0
        self.arm_straight_front = False
        self.arm_straight_down = False
        self.rpy_sub = rospy.Subscriber('myo_imu_rpy', Float64MultiArray, self.rpy_cb, queue_size=10)
        self.last_emg_avg = 0.0
        self.last_emgs = [0.0, 0.0, 0.0, 0.0, 0.0] # Let's try with the last 5 vals
        self.emg_sub = rospy.Subscriber('myo_emg', EmgArray, self.emg_cb, queue_size=10)
        self.twist_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=10)


    def rpy_cb(self, data):
        # Yaw represents the rotation of the arm
        # 0.7 would be 0% volume
        min_yaw = -1.4
        # -1.4 would be 100% volume
        max_yaw = 0.7
        full_range = abs(min_yaw) + abs(max_yaw)
        yaw = data.data[2]
        volume = (yaw - min_yaw) / full_range
        # print("yaw: " + str(yaw))
        # print("min_yaw: " + str(min_yaw) + " max_yaw: " + str(max_yaw))
        # print("")
        self.curr_vol = volume

        pitch = data.data[1]
        # For front
        min_pitch = -0.3
        max_pitch = 0.3
        if max_pitch > pitch > min_pitch:
            self.arm_straight_front = True
        else:
            self.arm_straight_front = False

        # For down
        min_pitch = 1.0
        max_pitch = 1.5
        if max_pitch > pitch > min_pitch:
            self.arm_straight_down = True
        else:
            self.arm_straight_down = False

    def emg_cb(self, data):
        """
        :type data: EmgArray
        """
        self.last_emg_avg = np.average(data.data)
        self.new_data = True

    def run(self):
        min_avg = 250
        while not rospy.is_shutdown():
            if not self.new_data:
                rospy.sleep(0.05)
                continue
            self.new_data = False
            if self.last_emg_avg > min_avg:
                print(OKGREEN + "self.last_emg_avg: " + str(self.last_emg_avg) + ENDC)
            else:
                print(FAIL + "self.last_emg_avg: " + str(self.last_emg_avg) + ENDC)
            if self.arm_straight_front:
                print(OKGREEN + "self.arm_straight_front: " + str(self.arm_straight_front) + ENDC)
            else:
                print(FAIL + "self.arm_straight_front: " + str(self.arm_straight_front) + ENDC)
            if self.arm_straight_down:
                print(OKGREEN + "self.arm_straight_down: " + str(self.arm_straight_down) + ENDC)
            else:
                print(FAIL + "self.arm_straight_down: " + str(self.arm_straight_down) + ENDC)

            print("With volume var: " + str(self.curr_vol))

            if self.last_emg_avg > min_avg:
                t = Twist()
                direction = self.curr_vol - 0.5 # so it's in -0.5 <-> 0.5 range
                if self.arm_straight_front:
                    if self.last_emg_avg > 1000.0:
                        final_val = 1000.0
                    else:
                        final_val = self.last_emg_avg
                    t.linear.x = (final_val / 1000.0 ) * 1.5
                    t.angular.z = direction * 2.0
                elif self.arm_straight_down:
                    t.angular.z = direction * 2.0

                self.twist_pub.publish(t)

            rospy.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('myo_drive', anonymous=True)
    vg = DriveMyo()
    vg.run()

