from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID

from commands.reset_xy import ResetXY, ResetSwerveFront

class ConfigureTests:
    """
    This class is used to configure the tests.
    """

    def __init__(self, robot_container):
        """
        Initialize the tests class.
        """

        self.robotDrive = robot_container.robotDrive
        self.testChooser = robot_container.testChooser

    def configureTestCommands(self):
        """
        Configures the test commands for the robot. This is called in test mode.
        """
        self.testChooser.setDefaultOption("No Test", InstantCommand())