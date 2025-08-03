from typing import Self
from commands2 import Command, CommandScheduler, InstantCommand, RunCommand
import commands2
from wpilib import Field2d, Joystick, SendableChooser, SmartDashboard
from commands2.button import Trigger, JoystickButton
import xrp
from commands.forwardAuto import DriveForward
import constants
from subsystems.drivetrain import DriveTrain
from pathplannerlib.auto import PathPlannerAuto
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Translation2d

class RobotContainer:
    field: Field2d = Field2d()
    def __init__(self) -> None:
        self.driveTrain: DriveTrain = DriveTrain()
        self.configureBindings()
        self.autoChooser = SendableChooser()
        self.configureAutos()
        
    def configureAutos(self) -> None:
        self.autoChooser.setDefaultOption("Default", PathPlannerAuto("Drive Forward"))
        self.autoChooser.addOption("Test", PathPlannerAuto("Test"))
        self.autoChooser.addOption("None", commands2.WaitCommand(0.1))
        self.autoChooser.addOption("Turn Auto", PathPlannerAuto("Turn Right"))
        self.autoChooser.addOption("Pivot", PathPlannerAuto("Pivot"))
        SmartDashboard.putData(self.autoChooser)

    def configureBindings(self: Self) -> None:
        self.joystick: Joystick = Joystick(constants.ControllerConstants.kJoystickPort)

        self.dynamicForwardBtn: JoystickButton = JoystickButton(self.joystick, 1)
        self.dynamicBackwardBtn: JoystickButton = JoystickButton(self.joystick, 2)
        self.quasistaticForwardBtn: JoystickButton = JoystickButton(self.joystick, 3)
        self.quasistaticBackwardBtn: JoystickButton = JoystickButton(self.joystick, 4)
        
        self.drivePathBtn: JoystickButton = JoystickButton(self.joystick, 5)

        self.driveTrain.setDefaultCommand(self.driveTrain.telopCommand(
            lambda: -self.joystick.getRawAxis(1),
            lambda: -self.joystick.getRawAxis(0)
        ))

        self.dynamicForwardBtn.onTrue(self.driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        self.dynamicBackwardBtn.onTrue(self.driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        self.quasistaticForwardBtn.onTrue(self.driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        self.quasistaticBackwardBtn.onTrue(self.driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))

        self.drivePathBtn.onTrue(InstantCommand(
            lambda: CommandScheduler.getInstance().schedule(
                self.driveTrain.getForwardOTF(Translation2d(0.3, 0))
            )
        ))
        

    def getAutoRoutine(self) -> Command:
        return self.autoChooser.getSelected()