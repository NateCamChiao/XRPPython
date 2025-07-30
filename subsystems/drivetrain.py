import math
from turtle import position
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

        self.configureSysId()
        config: RobotConfig = RobotConfig.fromGUISettings()
        # Configure the AutoBuilder last
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

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(
                units.inchesToMeters(self.m_leftEncoder.getRate()),
                units.inchesToMeters(self.m_rightEncoder.getRate())
            )
        )
    def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
        # normalize it by dividing by max angular velocity
        tankSpeeds: DifferentialDriveWheelSpeeds = self.kinematics.toWheelSpeeds(speeds).__truediv__(DriveTrainConstants.kMaxAngularVelocity)
        self.differentialDrive.tankDrive(tankSpeeds.left, tankSpeeds.right)

    def shouldFlipPath(self) -> bool:
        return False

    def periodic(self) -> None:
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage())
        self.differentialOdometry.update(
            self.m_gyro.getRotation2d(), 
            units.inchesToMeters(self.m_leftEncoder.getDistance()),
            units.inchesToMeters(self.m_rightEncoder.getDistance())
        )
        robotcontainer.RobotContainer.field.setRobotPose(self.differentialOdometry.getPose())
        SmartDashboard.putData(robotcontainer.RobotContainer.field)
        command: Command | None = self.getCurrentCommand()
        SmartDashboard.putNumber("encoder positions", self.m_leftEncoder.getDistance())
        if SmartDashboard.getNumber("encoder rate", 0) < self.m_leftEncoder.getRate():
            SmartDashboard.putNumber("encoder rate", self.m_leftEncoder.getRate())

        if command != None:
            SmartDashboard.putString("current command", command.getName())
            
    def simulationPeriodic(self) -> None:
        ...
    # Resets encoder distance reading to 0
    def resetEncoders(self) -> None:
        self.m_leftEncoder.reset()
        self.m_rightEncoder.reset()
    
    def drive(self, forwardSpeed: float, angularVelocity: float) -> None:
        self.differentialDrive.arcadeDrive(forwardSpeed, angularVelocity)

    def getRate(self) -> float: # average speed (inch)
        return self.m_leftEncoder.getRate()

    def telopCommand(self, forwardSupplier: Callable[[], float], rotationSupplier: Callable[[], float]) -> RunCommand:
        return RunCommand(
            lambda: self.drive(forwardSupplier(), rotationSupplier()),
            self
        )
    # Setting up sysid here to avoid cluttering constructor
    def configureSysId(self) -> None:
        def driveMotors(voltage: float) -> None:
            self.m_leftMotor.setVoltage(voltage)
            self.m_rightMotor.setVoltage(voltage)
        
        def logData(log: SysIdRoutineLog) -> None:
            log.motor("left Motor") \
                .voltage(self.m_leftMotor.get() * RobotController.getBatteryVoltage()) \
                .position(units.inchesToMeters(self.m_leftEncoder.getDistance())) \
                .velocity(units.inchesToMeters(self.m_leftEncoder.getRate()))
            log.motor("right Motor") \
                .voltage(self.m_rightMotor.get() * RobotController.getBatteryVoltage()) \
                .position(units.inchesToMeters(self.m_rightEncoder.getDistance())) \
                .velocity(units.inchesToMeters(self.m_rightEncoder.getRate()))
  
        self.sysIdRoutine: SysIdRoutine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                lambda volts: driveMotors(volts),
                lambda sysIdRoutineLog: logData(sysIdRoutineLog),
                self,
                "Drivetrain"
            )
        )
    
    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysIdRoutine.dynamic(direction)
    
    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysIdRoutine.quasistatic(direction)