import math

################################################################
## Additional Unit conversion classes not in wpilib.units


def deg2Rad(inVal):
    return math.pi / 180.0 * inVal


def rad2Deg(inVal):
    return 180.0 / math.pi * inVal


def rev2Rad(inVal):
    return 2.0 * math.pi * inVal


def rad2Rev(inVal):
    return 1.0 / (math.pi * 2.0) * inVal


def m2ft(inVal):
    return 3.28084 * inVal


def ft2m(inVal):
    return float(inVal / 3.28084)


def m2in(inVal):
    return 39.3701 * inVal


def in2m(inVal):
    return inVal / 39.3701


def radPerSec2RPM(inVal):
    return inVal * 9.55


# pylint: disable=invalid-name
def RPM2RadPerSec(inVal):
    return inVal / 9.55


def wrapAngleDeg(angle):
    angle %= 360.0
    angle = (angle - 360) if angle > 180 else angle
    angle = (angle + 360) if angle < -180 else angle
    return angle


def wrapAngleRad(angle):
    return deg2Rad(wrapAngleDeg(rad2Deg(angle)))


def lbsToKg(lbs_in):
    return 0.4535924 * lbs_in

def sign(val_in):
    if(val_in > 0):
        return 1.0
    elif(val_in < 0):
        return -1.0
    else:
        return 0.0