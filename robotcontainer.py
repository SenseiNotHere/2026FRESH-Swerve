from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SmartDashboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from constants import autoConstants, DriveConstants, OIConstants, LiftConstants
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics, AutoBuilder
from subsystems.limelight_camera import LimelightCamera
from subsystems.elevator import Elevator
from buttonbindings import ButtonBindings

from commands.reset_xy import ResetXY, ResetSwerveFront

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)
        self.limelight =  LimelightCamera("limelight")

        # The driver's controller
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings = ButtonBindings.configureButtonBindings
        self.configureButtonBindings(self)

        # Configure default command for driving using sticks
        from commands.holonomicdrive import HolonomicDrive
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )

        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.autoChooser.getSelected()
        if command is None:
            print("WARNING: No autonomous selected!")
            # Return a command that does nothing
            return commands2.InstantCommand()

        print(f"Running auto: {command.getName()}")
        return command


    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
