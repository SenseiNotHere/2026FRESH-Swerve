import math
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import ModuleConstants

class PhoenixSwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            chassisAngularOffset: float,
            turnMotorInverted: bool = True,
    ) -> None:
        """
        :param drivingCANId: The CAN ID for the driving motor.
        :param turningCANId: The CAN ID for the turning motor.
        :param chassisAngularOffset: Offset angle for this module.
        :param turnMotorInverted: Whether the turning motor is inverted or not.
        """

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        #Initialize the TalonFX Controllers
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)

        #Initialize Driving Motors
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # Set appropriate PID values for velocity control
        drivingConfig.slot0.kP = 0.1
        drivingConfig.slot0.kI = 0.0
        drivingConfig.slot0.kD = 0.0
        self.drivingMotor.configurator.apply(drivingConfig)

        #Initialize Turning Motors
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        #Set appropriate PID values for position control
        turningConfig.slot0.kP = 50.0
        turningConfig.slot0.kI = 0.0
        turningConfig.slot0.kD = 0.1
        #Use InvertedValue enum instead of bool
        turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.turningMotor.configurator.apply(turningConfig)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        velocity = self.drivingMotor.get_velocity().value
        angle = self._getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        distance = self.drivingMotor.get_position().value
        angle = self._getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(distance, Rotation2d(angle))

    def _getTurningPosition(self) -> float:
        """Gets the turning motor position in radians."""
        # Convert rotations to radians (2π radians per rotation)
        return self.turningMotor.get_position().value * 2 * math.pi

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module."""
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # If speed is too low, don't move, save power
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Apply chassis angular offset to the desired state
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )

        # Get the current angle for optimization
        current_angle = Rotation2d(self._getTurningPosition())

        # Manual optimization instead of using SwerveModuleState.optimize()
        angle_diff = current_angle.radians() - correctedDesiredState.angle.radians()

        # Normalize the angle difference to -π to π
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # If the difference is greater than 90 degrees (π/2), flip the direction
        if abs(angle_diff) > math.pi / 2:
            optimized_speed = -correctedDesiredState.speed
            optimized_angle = Rotation2d(correctedDesiredState.angle.radians() + math.pi)
        else:
            optimized_speed = correctedDesiredState.speed
            optimized_angle = correctedDesiredState.angle

        # Convert optimized angle to rotations
        angle_in_rotations = optimized_angle.radians() / (2 * math.pi)

        #Send commands to motors
        self.drivingMotor.set_control(self.velocity_request.with_velocity(optimized_speed))
        self.turningMotor.set_control(self.position_request.with_position(angle_in_rotations))

        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes the driving encoder."""
        self.drivingMotor.set_position(0)
