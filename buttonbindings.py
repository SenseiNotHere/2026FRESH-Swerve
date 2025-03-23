import commands2
from constants import OIConstants

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelight_camera import LimelightCamera


from commands.reset_xy import ResetXY, ResetSwerveFront

class ButtonBindings:
    def __init__(self, robot):
        self.robotDrive = DriveSubsystem()
        self.limelight =  LimelightCamera("limelight")

        #Driver Controller
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        #Driver Button Bindings
        driverPovUp = self.driverController.pov(0)
        driverPovUp.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))

        driverPovDown = self.driverController.pov(180)
        driverPovDown.onTrue(ResetSwerveFront(self.robotDrive))

        driverXButton = self.driverController.button(XboxController.Button.kX)
        driverXButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

