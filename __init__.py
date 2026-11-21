"""SO-101 Robot Arm Package."""
if __package__:
    from .kinematics import SO101Kinematics, ArmGeometry, SingularityStatus
    from .hardware import JointConfig, CalibrationStore, RobotInterface, SimRobot, FeetechRobot, ServoRange, SERVO_RANGE
    from .controller import MotionController
    from .gui import RobotStudioApp
else:
    from kinematics import SO101Kinematics, ArmGeometry, SingularityStatus
    from hardware import JointConfig, CalibrationStore, RobotInterface, SimRobot, FeetechRobot, ServoRange, SERVO_RANGE
    from controller import MotionController
    from gui import RobotStudioApp

__all__ = [
    "SO101Kinematics",
    "ArmGeometry",
    "SingularityStatus",
    "JointConfig",
    "CalibrationStore",
    "RobotInterface",
    "SimRobot",
    "FeetechRobot",
    "ServoRange",
    "SERVO_RANGE",
    "MotionController",
    "RobotStudioApp",
]
