import math

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from rev import SparkMax, SparkFlex, SparkLowLevel, SparkAbsoluteEncoder, SparkBase
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig

class HybridSwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            drivingControllerType,
            turningCANId: int,
            turningControllerType,
            chassisAngularOffset: float,
            turnMotorInverted: bool = True,
    ) -> None:
        """
        :param drivingCANId: The CAN ID for the driving motor.
        :param drivingControllerType: The type of motor controller to use for driving. (TalonFX, SparkFlex, or SparkMax)
        :param turningCANId: The CAN ID for the turning motor.
        :param turningControllerType: The type of motor controller to use for turning. (TalonFX, SparkFlex, or SparkMax)
        :param chassisAngularOffset: Offset angle for this module.
        :param turnMotorInverted: Whether the turning motor is inverted or not.
        """

        self.drivingControllerType = drivingControllerType
        self.turningControllerType = turningControllerType

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Initializing motors
        if drivingControllerType is TalonFX: #If Driving Motor is a TalonFX
            self.drivingMotor = TalonFX(drivingCANId)
        elif drivingControllerType is SparkFlex: #If Driving Motor is a SparkFlex
            self.drivingMotor = SparkFlex(
                drivingCANId, SparkLowLevel.MotorType.kBrushless
            )
        else: #If Driving Motor is a SparkMax
            self.drivingMotor = SparkMax(
                drivingCANId, SparkLowLevel.MotorType.kBrushless)
        #ONLY TalonFX, SparkFlex, and SparkMax are supported for driving motors using this class

        if turningControllerType is TalonFX: #If Turning Motor is a TalonFX
            self.turningMotor = TalonFX(turningCANId)
        elif turningControllerType is SparkFlex: #If Turning Motor is a SparkFlex
            self.turningMotor = SparkFlex(
                turningCANId, SparkLowLevel.MotorType.kBrushless
            )
        else: #If Turning Motor is a SparkMax
            self.turningMotor = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)
        #ONLY TalonFX, SparkFlex, and SparkMax are supported for turning motors using this class

        if drivingControllerType is TalonFX: #Configure TalonFX Driving Motor, if Driving Motor is a TalonFX
            drivingConfig = TalonFXConfiguration()
            drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
            # Set appropriate PID values for velocity control
            drivingConfig.slot0.kP = 0.1
            drivingConfig.slot0.kI = 0.0
            drivingConfig.slot0.kD = 0.0
            self.drivingMotor.configurator.apply(drivingConfig)

        if turningControllerType is TalonFX: #Configure TalonFX Turning Motor, if Turning Motor is a TalonFX
            turningConfig = TalonFXConfiguration()
            turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
            # Set appropriate PID values for position control
            turningConfig.slot0.kP = 50.0
            turningConfig.slot0.kI = 0.0
            turningConfig.slot0.kD = 0.1
            # Use InvertedValue enum instead of bool
            turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            self.turningMotor.configurator.apply(turningConfig)

        if drivingControllerType is SparkMax or drivingControllerType is SparkFlex:
            self.drivingMotor.configure(
                getSwerveDrivingMotorConfig(),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

            self.drivingEncoder = self.drivingMotor.getEncoder()
            self.drivingPIDController = self.drivingMotor.getClosedLoopController()

        if turningControllerType is SparkMax or turningControllerType is SparkFlex:
            self.turningMotor.configure(
                getSwerveTurningMotorConfig(turnMotorInverted),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters)

            self.turningEncoder = self.turningMotor.getAbsoluteEncoder()
            self.turningPIDController = self.turningMotor.getClosedLoopController()

        self.chassisAngularOffset = chassisAngularOffset
        if turningControllerType is SparkMax or turningControllerType is SparkFlex:
            self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        elif turningControllerType is TalonFX:
            self.desiredState.angle = Rotation2d(self.turningMotor.get_position().value)

            if drivingControllerType is SparkMax or drivingControllerType is SparkFlex:
                self.drivingEncoder.setPosition(0)
            elif drivingControllerType is TalonFX:
                self.drivingMotor.set_position(0)

    def getState(
            self,
            drivingControllerType,
            turningControllerType,
    ) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        if drivingControllerType is TalonFX:
            velocity = self.drivingMotor.get_velocity().value
        elif drivingControllerType is SparkMax or drivingControllerType is SparkFlex:
            velocity = self.drivingEncoder.getVelocity()
        else:
            raise ValueError("Unsupported driving motor type")

        if turningControllerType is TalonFX:
            angle = self._getTurningPosition() - self.chassisAngularOffset
        elif turningControllerType is SparkMax or turningControllerType is SparkFlex:
            angle = self.turningEncoder.getPosition() - self.chassisAngularOffset
        else:
            raise ValueError("Unsupported turning motor type")

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self):
        """Returns the current position of the module.

        :returns: The current position of the module.
        """

        if self.drivingControllerType is TalonFX:
            distance = self.drivingMotor.get_position().value
        elif self.drivingControllerType is SparkMax or self.drivingControllerType is SparkFlex:
            distance = self.drivingEncoder.getPosition()
        else:
            raise ValueError("Unsupported driving motor type")

        if self.turningControllerType is TalonFX:
            angle = self._getTurningPosition() - self.chassisAngularOffset
        elif self.turningControllerType is SparkMax or self.turningControllerType is SparkFlex:
            angle = self.turningEncoder.getPosition() - self.chassisAngularOffset
        else:
            raise ValueError("Unsupported turning motor type")

        return SwerveModulePosition(distance, Rotation2d(angle))

    def _getTurningPosition(self):
        """Gets the turning motor position in radians."""
        if self.turningControllerType is TalonFX:
            return self.turningMotor.get_position().value
        elif self.turningControllerType is SparkMax or self.turningControllerType is SparkFlex:
            return self.turningEncoder.getPosition()
        else:
            raise ValueError("Unsupported turning motor type")

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module."""

        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        optimizedDesiredState = SwerveModuleState()

        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassisAngularOffset)

        if self.turningControllerType is SparkMax or self.turningControllerType is SparkFlex:
            current_angle = Rotation2d(self.turningEncoder.getPosition())

            result = SwerveModuleState.optimize(correctedDesiredState, current_angle)
            if result is not None:
                optimizedDesiredState = result
            else:
                #Fallback if optimize returns None
                optimizedDesiredState = correctedDesiredState

            self.turningPIDController.setReference(
                optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition)

        elif self.turningControllerType is TalonFX:
            current_angle = Rotation2d(self._getTurningPosition())
            angle_diff = current_angle.radians() - correctedDesiredState.angle.radians()
            #Normalize angle (-π to π)
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            if abs(angle_diff) > math.pi / 2:
                optimizedDesiredState = SwerveModuleState(
                    -correctedDesiredState.speed,
                    Rotation2d(correctedDesiredState.angle.radians() + math.pi))
            else:
                optimizedDesiredState = correctedDesiredState

            angle_in_rotations = optimizedDesiredState.angle.radians() / (2 * math.pi)
            self.turningMotor.set_control(self.position_request.with_position(angle_in_rotations))

        if self.drivingControllerType is SparkMax or self.drivingControllerType is SparkFlex:
            self.drivingPIDController.setReference(
                optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity)
        elif self.drivingControllerType is TalonFX:
            self.drivingMotor.set_control(self.velocity_request.with_velocity(optimizedDesiredState.speed))

        self.desiredState = desiredState

    def stop(self):
        """
        Stops the module in place to conserve energy and avoid unnecessary brownouts
        """
        if self.drivingControllerType is SparkMax or self.drivingControllerType is SparkFlex:
            self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        elif self.drivingControllerType is TalonFX:
            self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        else:
            raise ValueError("Unsupported driving motor type")

        if self.turningControllerType is SparkMax or self.turningControllerType is SparkFlex:
            self.turningPIDController.setReference(self.turningEncoder.getPosition(), SparkLowLevel.ControlType.kPosition)
        elif self.turningControllerType is TalonFX:
            current_position = self.turningMotor.get_position().value
            self.turningMotor.set_control(self.position_request.with_position(current_position))
        else:
            raise ValueError("Unsupported turning motor type")

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes the driving encoder."""
        if self.drivingControllerType is TalonFX:
            self.drivingMotor.set_position(0)
        elif self.drivingControllerType is SparkMax or self.drivingControllerType is SparkFlex:
            self.drivingEncoder.setPosition(0)
        else:
            raise ValueError("Unsupported driving motor type")