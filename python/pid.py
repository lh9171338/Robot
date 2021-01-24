import rospy
import numpy as np


class PidParam:
    def __init__(self, KP=0, KI=0, KD=0, duration=0, windup=0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.duration = duration
        self.windup = windup


class Pid:
    def __init__(self, pidParam):
        self.pidParam = pidParam
        self.PItem = 0
        self.IItem = 0
        self.DItem = 0
        self.last_error = 0
        self.last_time = rospy.Time().now().to_sec()
        self.output = None

    def CalcPid(self, input, target):
        # Calculate delta time
        cur_time = rospy.Time().now().to_sec()
        delta_time = cur_time - self.last_time
        if delta_time >= self.pidParam.duration:
            # Calculate P item
            error = target - input
            self.PItem = error

            # Calculate I item
            self.IItem += error * delta_time
            self.IItem = np.clip(self.IItem, -self.pidParam.windup, self.pidParam.windup)

            # Calculate D item
            self.DItem = (error - self.last_error) / delta_time

            # Calculate output
            self.output = self.pidParam.KP * self.PItem + self.pidParam.KI * self.IItem + self.pidParam.KD * self.DItem

            # Update
            self.last_error = error
            self.last_time = cur_time

        return self.output