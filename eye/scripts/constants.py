from enum import IntEnum

class Action(IntEnum):
    reset = 0
    captureImage = 1
    detectFiducial = 2
    detectHole = 3
    pullRivet = 4