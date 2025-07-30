import wpimath.units as Units
class DriveTrainConstants:
    kLeftMotorID = 0
    kRightMotorID = 1
    kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0)  #  48.75:1
    kCountsPerMotorShaftRev = 12.0
    kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio  # 585.0
    kWheelDiameterInch = 2.3622  # 60 mm
    kTrackWidth = 0.16 # meters
    kMaxAngularVelocity: float = Units.inchesToMeters(23) # inch to meters 1 rotation per sec (23 inch per sec) 19.89 prev
    kMaxSpeedPerSecMeters: float = 0.653 # meters

class ControllerConstants:
    kJoystickPort = 0

class PathPlannerConstants:
    # Note: These go in "robot config" in pathplanner...Probably shouldn't be used in code
    # Writing these down for future reference
    # all length/width dimensions are in meters
    Robot_Mass = 0.428 # kg
    Robot_MOI = 0.003 # kg * M ^2
    Track_width = 0.16 # distance from wheels
    # Bumper settings are not required. They're used to visually indicate the size of the robot when creating autos
    Bumper_width = 0.18
    Bumper_length = 0.19
    True_max_drive_speed = 0.653 # M / S
    Drive_gearing = 48.75
    Wheel_COF = 0.5 # This is a really rough estimate 
    Drive_motor = "MiniCIM" # Doesn't seem to affect anything so use whatever
    Drive_current_limit = 5 # Really not sure with this one
