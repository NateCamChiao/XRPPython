import math
from typing import Callable

from commands2 import Command, Subsystem, RunCommand
from wpilib import Encoder, RobotController, SmartDashboard
from wpilib.drive import DifferentialDrive
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from xrp import XRPGyro, XRPMotor
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from constants import DriveTrainConstants
import robotcontainer
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
from wpimath import units

class DriveTrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.m_leftMotor: XRPMotor = XRPMotor(0)
        self.m_rightMotor: XRPMotor = XRPMotor(1)
        self.m_rightMotor.setInverted(True)
        # ports are absolute for encoder
        self.m_leftEncoder: Encoder = Encoder(4, 5)
        self.m_rightEncoder: Encoder = Encoder(6, 7)

        self.differentialDrive: DifferentialDrive = DifferentialDrive(self.m_leftMotor.set, self.m_rightMotor.set)
        # setting up encoders
        self.m_leftEncoder.setDistancePerPulse((math.pi * DriveTrainConstants.kWheelDiameterInch) / DriveTrainConstants.kCountsPerRevolution)
        self.m_rightEncoder.setDistancePerPulse((math.pi * DriveTrainConstants.kWheelDiameterInch) / DriveTrainConstants.kCountsPerRevolution)

        self.resetEncoders()

        self.m_gyro: XRPGyro = XRPGyro()
        self.differentialOdometry: DifferentialDriveOdometry = DifferentialDriveOdometry(self.m_gyro.getRotation2d(), 0, 0)
        self.kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(DriveTrainConstants.kTrackWidth)
        # Separate function to configure sysid
        self.configureSysId()
        config: RobotConfig = RobotConfig.fromGUISettings()
        # Configure the AutoBuilder last in constructor
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPLTVController(0.02), # PPLTVController is the built in path following controller for differential drive trains
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def getPose(self) -> Pose2d:
        return self.differentialOdometry.getPose()

    def resetPose(self, newPose: Pose2d) -> None:
        self.differentialOdometry.resetPose(newPose)
    # Should use kinematics to convert to chassisSpeeds
    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
                units.inchesToMeters(self.m_leftEncoder.getRate()),
                units.inchesToMeters(self.m_rightEncoder.getRate())
            )
        )
    # Turns chassis speeds into wheel speeds and drives via tank drive
    def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
        # normalize it by dividing by max angular velocity
        tankSpeeds: DifferentialDriveWheelSpeeds = self.kinematics.toWheelSpeeds(speeds).__truediv__(DriveTrainConstants.kMaxAngularVelocity)
        self.differentialDrive.tankDrive(tankSpeeds.left, tankSpeeds.right)
    # If field is mirrored, then you need to add logic based on alliance color
    def shouldFlipPath(self) -> bool:
        return False

    def periodic(self) -> None:
        # Helpful info
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage())
        # Updating odometry to get semi-accurate pose estimation
        self.differentialOdometry.update(
            self.m_gyro.getRotation2d(), 
            units.inchesToMeters(self.m_leftEncoder.getDistance()),
            units.inchesToMeters(self.m_rightEncoder.getDistance())
        )
        # updating virtual field
        robotcontainer.RobotContainer.field.setRobotPose(self.differentialOdometry.getPose())
        SmartDashboard.putData(robotcontainer.RobotContainer.field)
        # Usefull for determining what command is being run on drivetrain
        command: Command | None = self.getCurrentCommand()
        if command != None:
            SmartDashboard.putString("current command", command.getName())

    # Resets encoder distance reading to 0
    def resetEncoders(self) -> None:
        self.m_leftEncoder.reset()
        self.m_rightEncoder.reset()
    # Drive drivetrain using arcade drive (used in telop)
    def drive(self, forwardSpeed: float, angularVelocity: float) -> None:
        self.differentialDrive.arcadeDrive(forwardSpeed, angularVelocity)
    # Returns a command for driving teleop. This is used in the default command
    def telopCommand(self, forwardSupplier: Callable[[], float], rotationSupplier: Callable[[], float]) -> RunCommand:
        return RunCommand(
            lambda: self.drive(forwardSupplier(), rotationSupplier()),
            self
        )
    # Setting up sysid here to avoid cluttering constructor
    def configureSysId(self) -> None:
        # function to supply voltage to motors
        def driveMotors(voltage: float) -> None:
            self.m_leftMotor.setVoltage(voltage)
            self.m_rightMotor.setVoltage(voltage)
        # function to log sensor values: voltage, position, and velocity
        def logData(log: SysIdRoutineLog) -> None:
            log.motor("left Motor") \
                .voltage(self.m_leftMotor.get() * RobotController.getBatteryVoltage()) \
                .position(units.inchesToMeters(self.m_leftEncoder.getDistance())) \
                .velocity(units.inchesToMeters(self.m_leftEncoder.getRate()))
            log.motor("right Motor") \
                .voltage(self.m_rightMotor.get() * RobotController.getBatteryVoltage()) \
                .position(units.inchesToMeters(self.m_rightEncoder.getDistance())) \
                .velocity(units.inchesToMeters(self.m_rightEncoder.getRate()))
        # This creates a sysIdRoutine object that holds a config and mechanis
        self.sysIdRoutine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(), # Default configuration
            SysIdRoutine.Mechanism( 
                lambda volts: driveMotors(volts), # pass in drive function
                lambda sysIdRoutineLog: logData(sysIdRoutineLog), # pass in log functions
                self,
                "Drivetrain" # mechanism name
            )
        )
    # this returns a command that commands robot to follow dynamic routine
    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysIdRoutine.dynamic(direction)
    # this returns a command that commands robot to follow quasistatic routine
    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysIdRoutine.quasistatic(direction)