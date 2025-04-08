from wpilib import XboxController, PS4Controller
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID

from commands.reset_xy import ResetXY, ResetSwerveFront


class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container
        """
        self.robotDrive = robot_container.robotDrive
        self.driverController = robot_container.driverController

    def configureButtonBindings(self):
        """Configure button bindings for the robot."""

        # Driver Controls
        povUpDriverButton = self.driverController.pov(0)
        povUpDriverButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        povUpDriverButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))

        povDownDriverButton = self.driverController.pov(180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))