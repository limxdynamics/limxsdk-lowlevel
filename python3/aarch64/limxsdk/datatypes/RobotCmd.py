﻿import sys

class RobotCmd(object):
    __slots__ = ['stamp','mode','q','dq','tau','Kp','Kd', 'motor_names']
    def __init__(self):
        self.stamp = 0
        self.mode = []
        self.q = []
        self.dq = []
        self.tau = []
        self.Kp = []
        self.Kd = []
        self.motor_names = []