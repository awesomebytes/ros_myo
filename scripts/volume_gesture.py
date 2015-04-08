#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from ros_myo.msg import EmgArray
import numpy as np

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


class VolumeGesture():
    """
    """
    def __init__(self):
        self.curr_vol = 0.0
        self.arm_straight_front = False
        self.rpy_sub = rospy.Subscriber('myo_imu_rpy', Float64MultiArray, self.rpy_cb, queue_size=10)
        self.last_emg_avg = 0.0
        self.last_emgs = [0.0, 0.0, 0.0, 0.0, 0.0] # Let's try with the last 5 vals
        self.emg_sub = rospy.Subscriber('myo_emg', EmgArray, self.emg_cb, queue_size=10)


    def rpy_cb(self, data):
        # Yaw represents the rotation of the arm
        # 0.7 would be 0% volume
        min_yaw = 0.7
        # -1.4 would be 100% volume
        max_yaw = -1.4
        yaw = data.data[2]
        volume = (yaw - min_yaw) / max_yaw
        self.curr_vol = volume

        pitch = data.data[1]
        min_pitch = -0.3
        max_pitch = 0.3
        if max_pitch > pitch > min_pitch:
            self.arm_straight_front = True
        else:
            self.arm_straight_front = False

    def emg_cb(self, data):
        """
        :type data: EmgArray
        """
        self.last_emg_avg = np.average(data.data)

    def run(self):

        while not rospy.is_shutdown():
            print("self.last_emg_avg: " + str(self.last_emg_avg))
            if self.arm_straight_front:
                print(OKGREEN + "self.arm_straight_front: " + str(self.arm_straight_front) + ENDC)
            else:
                print(FAIL + "self.arm_straight_front: " + str(self.arm_straight_front) + ENDC)
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('myo_volume_gesture_node', anonymous=True)
    vg = VolumeGesture()
    vg.run()

