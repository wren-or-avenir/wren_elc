from .status import GPIN

class Pen:
    def __init__(self, pin, yaw_allow=2.0, pitch_allow=2.0, hysteresis=0.5,active_low = True):
        self.status = GPIN()
        self.yaw_allow = yaw_allow
        self.pitch_allow = pitch_allow
        self.active_low = active_low
        self.hysteresis = hysteresis
        