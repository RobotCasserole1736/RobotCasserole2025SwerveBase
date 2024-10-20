import sys
import wpilib
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.calibration import CalibrationWrangler
from utils.faults import FaultWrangler
from utils.crashLogger import CrashLogger
from utils.rioMonitor import RIOMonitor
from utils.singleton import destroyAllSingletonInstances
from utils.powerMonitor import PowerMonitor
from webserver.webserver import Webserver
from AutoSequencerV2.autoSequencer import AutoSequencer
from utils.powerMonitor import PowerMonitor
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        self.crashLogger = CrashLogger()
        wpilib.LiveWindow.disableAllTelemetry()
        self.webserver = Webserver()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()      

        self.dInt = DriverInterface()
        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        self.rioMonitor = RIOMonitor()
        self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.addPeriodic(FaultWrangler().update, 0.2, 0.0)

        self.autoHasRun = False


    def robotPeriodic(self):
        self.stt.start()

        self.dInt.update()
        self.stt.mark("Driver Interface")

        self.driveTrain.update()
        self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst.telemetry.setWPITrajectory(self.autodrive.getTrajectory())
        self.driveTrain.poseEst.telemetry.setCurTrajWaypoints(self.autodrive.getWaypoints())
        self.driveTrain.poseEst.telemetry.setCurObstacles(self.autodrive.getObstacles())
        self.stt.mark("Telemetry")

        self.ledCtrl.setAutoDrive(self.autodrive.isRunning())
        self.ledCtrl.setStuck(self.autodrive.rfp.isStuck())
        self.ledCtrl.update()
        self.stt.mark("LED Ctrl")

        logUpdate()
        self.stt.end()

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self):

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous rouines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())

        # Mark we at least started autonomous
        self.autoHasRun = True

    def autonomousPeriodic(self):

        self.autoSequencer.update()

        # Operators cannot control in autonomous
        self.driveTrain.setManualCmd(DrivetrainCommand())

    def autonomousExit(self):
        self.autoSequencer.end()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self):
        # clear existing telemetry trajectory
        self.driveTrain.poseEst.telemetry.setWPITrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(
                Pose2d(1.0, 1.0, Rotation2d(0.0))
            )


    def teleopPeriodic(self):

        # TODO - this is technically one loop delayed, which could induce lag
        # Probably not noticeable, but should be corrected.
        self.driveTrain.setManualCmd(self.dInt.getCmd())

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()

        if self.dInt.getCreateObstacle():
            # For test purposes, inject a series of obstacles around the current pose
            ct = self.driveTrain.poseEst.getCurEstPose().translation()
            tfs = [
                Translation2d(1.7, -0.5),
                Translation2d(0.75, -0.75),
                Translation2d(1.7, 0.5),
                Translation2d(0.75, 0.75),
                Translation2d(2.0, 0.0),
                Translation2d(0.0, 1.0),
                Translation2d(0.0, -1.0),
            ]
            for tf in tfs:
                obs = PointObstacle(location=(ct+tf), strength=0.7)
                self.autodrive.rfp.addObstacleObservation(obs)

        self.autodrive.setRequest(self.dInt.getNavToSpeaker(), self.dInt.getNavToPickup())

        # No trajectory in Teleop
        Trajectory().setCmd(None)

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajCtrl.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    #########################################################
    ## Test-Specific init and update
    def testInit(self):
        wpilib.LiveWindow.setEnabled(False)

    def testPeriodic(self):
        pass

    #########################################################
    ## Cleanup
    def endCompetition(self):
        self.rioMonitor.stopThreads()
        destroyAllSingletonInstances()
        super().endCompetition()

def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()

