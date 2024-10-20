from utils.calibration import Calibration
from utils.units import RPM2RadPerSec


class SwerveModuleGainSet:
    """Helper class to house all calibrated gains for one swerve drive module.
    Currently, this includes
     * Wheel feed back PID gains
     * Wheel feed-forward SVA gains
     * Azimuth feed back PID gains
    """

    def __init__(self):

        self.wheelP = Calibration("Drivetrain Module Wheel kP", 0.00005)
        self.wheelI = Calibration("Drivetrain Module Wheel kI", 0.0)
        self.wheelD = Calibration("Drivetrain Module Wheel kD", 0.0)
        self.wheelA = Calibration(
            "Drivetrain Module Wheel kA", 0.000, "volts/radPerSecPerSec"
        )
        self.wheelV = Calibration(
            "Drivetrain Module Wheel kV", 12.0 / RPM2RadPerSec(4700), "volts/radPerSec"
        )
        self.wheelS = Calibration("Drivetrain Module Wheel kS", 0.12, "volts")
        self.azmthP = Calibration("Drivetrain Module Azmth kP", 0.115)
        self.azmthI = Calibration("Drivetrain Module Azmth kI", 0.0)
        self.azmthD = Calibration("Drivetrain Module Azmth kD", 0.0000)

    def hasChanged(self)->bool:
        """
        Returns:
            bool: True if any gain in the set is modified, false otherwise
        """
        return (
            self.wheelP.isChanged()
            or self.wheelI.isChanged()
            or self.wheelD.isChanged()
            or self.wheelA.isChanged()
            or self.wheelV.isChanged()
            or self.wheelS.isChanged()
            or self.azmthP.isChanged()
            or self.azmthI.isChanged()
            or self.azmthD.isChanged()
        )
