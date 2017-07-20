from BrickPi import *

class RobotBehaviour:
    def __init__(self):
        pass

    def obstruction(self):
        distance = BrickPi.Sensor[sensor1]
        print distance
        return distance
