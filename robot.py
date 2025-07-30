import commands2
from ntcore import NetworkTableInstance
import wpilib
from commands2 import InstantCommand
from wpilib import Field2d, SmartDashboard
from robotcontainer import RobotContainer
import os
"""
VERY IMPORTANT TO HAVE os.environ
"""
os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"
class MyRobot(commands2.TimedCommandRobot):
    field: Field2d = Field2d()
    def __init__(self, period: float = wpilib.TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)
        self.autonomousCommand = InstantCommand()
        
    def robotInit(self) -> None:
        super().robotInit()
        self.m_robotContainer: RobotContainer = RobotContainer()
    # def disabledInit(self) -> None:
    #     """This function is called once each time the robot enters Disabled mode."""

    # def disabledPeriodic(self) -> None:
    #     """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand: commands2.Command = self.m_robotContainer.getAutoRoutine()
        SmartDashboard.putString("Auto Command", self.autonomousCommand.getName())
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    # def autonomousPeriodic(self) -> None:
    #     """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous command stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    # def teleopPeriodic(self) -> None:
    #     """This function is called periodically during operator control"""

    # def testInit(self) -> None:
    #     # Cancels all running commands at the start of test mode
    #     commands2.CommandScheduler.getInstance().cancelAll()
