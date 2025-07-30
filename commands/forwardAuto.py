from commands2 import Command
import wpilib
from subsystems.drivetrain import DriveTrain
class DriveForward(Command):
    """
    Simple auto that will drive at 50% speed for 0.5 seconds
    """
    def __init__(self, driveSub: DriveTrain) -> None:
        self.driveSub: DriveTrain = driveSub
        self.addRequirements(driveSub)
        self.timer: wpilib.Timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()
        self.driveSub.drive(0,0)

    def execute(self) -> None:
        self.driveSub.drive(0.5, 0)

    def isFinished(self) -> bool:
        return self.timer.get() > 0.5
    
    def end(self, interrupted: bool) -> None:
        self.driveSub.drive(0,0)
        self.timer.stop()
    