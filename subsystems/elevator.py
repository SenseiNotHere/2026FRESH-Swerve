from __future__ import annotations

from phoenix6.hardware import TalonFX
from phoenix6.configs import (
    TalonFXConfiguration,
    MotorOutputConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    Slot0Configs
)
from phoenix6.signals import (
    NeutralModeValue,
    ForwardLimitTypeValue,
    ReverseLimitTypeValue,
    InvertedValue
)
from phoenix6.controls import PositionVoltage, DutyCycleOut, Follower
from wpilib import SmartDashboard
from commands2 import Subsystem

# constants right here, to simplify
class ElevatorConstants:
    # very scary setting! (if set wrong, the arm will escape equilibrium and break something)
    absoluteEncoderInverted = False

    # if using relative encoder, how many motor revolutions are needed to move the elevator by one inch?
    GEAR_RATIO = 25
    PI = 3.1416
    GEAR_DIAMETER = 2.0
    motorRevolutionsPerInch = GEAR_RATIO / (GEAR_DIAMETER * PI)

    # if using absolute encoder on output shaft, how many output shaft revolutions needed to move elevator by an inch?
    absEncoderRevolutionsPerInch = motorRevolutionsPerInch / GEAR_RATIO  # is gear ratio == 20?

    # other settings
    leadMotorInverted = True
    followMotorInverted = False
    findingZeroSpeed = 0.1

    # calibrating? (at first, set it =True and calibrate all the constants above)
    calibrating = False

    # which range of motion we want from this elevator? (inside what's allowed by limit switches)
    minPositionGoal = 0.5  # inches
    maxPositionGoal = 32  # inches
    positionTolerance = 0.2  # inches

    # if we have an arm, what is the minimum and maximum safe angle for elevator to move
    # (we don't want to move with arm extended unsafely)
    minArmSafeAngleDegrees = 15
    maxArmSafeAngleDegrees = 80

    # PID configuration (after you are done with calibrating=True)
    kP = 0.02  # at first make it very small like this, then start tuning by increasing from there
    kD = 0.0  # at first start from zero, and when you know your kP you can start increasing kD from some small value >0
    kStaticGain = 0  # make it 3.5?
    kMaxOutput = 1.0


class Elevator(Subsystem):
    def __init__(
            self,
            leadMotorCANId: int,
            followMotorCANId: int | None = None,
            presetSwitchPositions: tuple = (),
            useAbsoluteEncoder: bool = False,
            limitSwitchNormallyOpen: bool = False,
            arm=None,
    ) -> None:
        """
        Constructs an elevator.
        Please be very, very careful with setting kP and kD in ElevatorConstants (elevators are dangerous)
        :param arm: if you want elevator to freeze and not move at times when this arm is in unsafe positions
        """
        super().__init__()

        self.zeroFound = False
        self.positionGoal = None
        self.positionGoalSwitchIndex = 0
        self.presetSwitchPositions = presetSwitchPositions

        # do we have an arm what we must watch for safe angles?
        self.arm = arm
        self.unsafeToMove = ""  # empty string = not unsafe
        self.armUnsafeFreezePositionGoal = None

        # initialize the motors
        self.leadMotor = TalonFX(leadMotorCANId)
        config = self._getLeadMotorConfig(
            inverted=ElevatorConstants.leadMotorInverted,
            limitSwitchNormallyOpen=limitSwitchNormallyOpen,
            positionFactor=1.0 / ElevatorConstants.motorRevolutionsPerInch,
            useAbsEncoder=useAbsoluteEncoder
        )
        self.leadMotor.configurator.apply(config)

        if followMotorCANId is not None:
            self.followMotor = TalonFX(followMotorCANId)
            # Configure follower
            follow_config = TalonFXConfiguration()
            motor_output = MotorOutputConfigs()
            motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE if ElevatorConstants.leadMotorInverted != ElevatorConstants.followMotorInverted else InvertedValue.CLOCKWISE_POSITIVE
            motor_output.neutral_mode = NeutralModeValue.BRAKE
            follow_config.motor_output = motor_output
            self.followMotor.configurator.apply(follow_config)
            # Use Follower control object to follow the lead motor
            follower_request = Follower(leadMotorCANId, oppose_master_direction=ElevatorConstants.leadMotorInverted != ElevatorConstants.followMotorInverted)
            self.followMotor.set_control(follower_request)

        # Control requests
        self.position_request = PositionVoltage(0).with_slot(0)
        self.duty_cycle_request = DutyCycleOut(0)

        # Set the initial elevator goal
        goal = ElevatorConstants.minPositionGoal
        if useAbsoluteEncoder:
            goal = self.getPosition()
            self.zeroFound = True  # if using absolute encoder, zero is already found

        self.setPositionGoal(goal)

    def switchDown(self):
        if self.presetSwitchPositions:
            self.positionGoalSwitchIndex = self.positionGoalSwitchIndex - 1
            self.positionGoalSwitchIndex = max([self.positionGoalSwitchIndex, 0])
            self.setPositionGoal(self.presetSwitchPositions[self.positionGoalSwitchIndex])

    def switchUp(self):
        if self.presetSwitchPositions:
            self.positionGoalSwitchIndex = self.positionGoalSwitchIndex + 1
            self.positionGoalSwitchIndex = min([self.positionGoalSwitchIndex, len(self.presetSwitchPositions) - 1])
            self.setPositionGoal(self.presetSwitchPositions[self.positionGoalSwitchIndex])

    def setPositionGoal(self, goalInches: float) -> None:
        if self.unsafeToMove:
            return

        if goalInches < ElevatorConstants.minPositionGoal:
            goalInches = ElevatorConstants.minPositionGoal
        if goalInches > ElevatorConstants.maxPositionGoal:
            goalInches = ElevatorConstants.maxPositionGoal
        self.positionGoal = goalInches

        if self.zeroFound:
            # Use position control with TalonFX
            self.position_request.position = goalInches + ElevatorConstants.kStaticGain
            self.leadMotor.set_control(self.position_request)

    def getPositionGoal(self) -> float:
        return self.positionGoal

    def getPosition(self) -> float:
        # Get position from TalonFX integrated sensor
        return self.leadMotor.get_position().value

    def isDoneMoving(self) -> bool:
        return abs(self.positionGoal - self.getPosition()) <= ElevatorConstants.positionTolerance

    def getVelocity(self) -> float:
        # Get velocity from TalonFX integrated sensor
        return self.leadMotor.get_velocity().value

    def stopAndReset(self) -> None:
        self.leadMotor.set_control(self.duty_cycle_request.with_output(0))
        self.leadMotor.configurator.apply_reset()
        if hasattr(self, 'followMotor') and self.followMotor is not None:
            self.followMotor.set_control(self.duty_cycle_request.with_output(0))
            self.followMotor.configurator.apply_reset()

    def drive(self, speed, deadband=0.1, maxSpeedInchesPerSecond=5):
        # 1. driving is not allowed in these situations
        if not self.zeroFound and not ElevatorConstants.calibrating:
            return  # if we aren't calibrating, zero must be found first (before we can drive)
        if self.unsafeToMove:
            return  # driving is not allowed if arm is at an unsafe angle

        # 2. speed is assumed to be between -1.0 and +1.0, with a deadband
        if abs(speed) < deadband:
            speed = 0
        speed = speed * abs(speed)  # quadratic scaling, easier for humans

        # 3. use the speed to drive
        if not self.zeroFound:
            # Direct control with duty cycle
            self.leadMotor.set_control(self.duty_cycle_request.with_output(speed))
        elif speed != 0:  # if we have a PID controller, we control the position goal instead
            self.setPositionGoal(self.positionGoal + speed * maxSpeedInchesPerSecond / 50.0)  # we have 50 decisions/sec

    def findZero(self):
        # Did we find the zero previously?
        if self.zeroFound:
            return
        # Is it unsafe to move?
        if ElevatorConstants.calibrating or self.unsafeToMove:
            return

        # Check if limit switches are triggered
        reverse_limit = self.leadMotor.get_reverse_limit().value
        forward_limit = self.leadMotor.get_forward_limit().value

        # Did we find the zero just now?
        if reverse_limit and not forward_limit:
            self.zeroFound = True
            self.leadMotor.set_control(self.duty_cycle_request.with_output(0))  # zero setpoint now
            self.leadMotor.set_position(0.0)  # Reset the encoder position
            self.setPositionGoal(ElevatorConstants.minPositionGoal)
            return

        # Otherwise, continue finding it
        self.leadMotor.set_control(self.duty_cycle_request.with_output(-ElevatorConstants.findingZeroSpeed))

    def getState(self) -> str:
        if self.unsafeToMove:
            return self.unsafeToMove

        forward_limit = self.leadMotor.get_forward_limit().value
        reverse_limit = self.leadMotor.get_reverse_limit().value

        if forward_limit:
            return "forward limit" if not reverse_limit else "both limits (CAN disconn?)"
        if reverse_limit:
            return "reverse limit"
        if not self.zeroFound:
            return "finding zero"
        # otherwise, everything is ok
        return "ok"

    def isUnsafeToMove(self):
        if self.arm is not None:
            angle = self.arm.getAngle()
            if angle < ElevatorConstants.minArmSafeAngleDegrees:
                return "arm angle too low"
            if angle > ElevatorConstants.maxArmSafeAngleDegrees:
                return "arm angle too high"
            angleGoal = self.arm.angleGoal
            if angleGoal < ElevatorConstants.minArmSafeAngleDegrees:
                return "arm anglegoal too low"
            if angleGoal > ElevatorConstants.maxArmSafeAngleDegrees:
                return "arm anglegoal too high"
        return ""

    def periodic(self):
        # 1. do we need to stop the elevator because arm is at unsafe angle?
        unsafeToMove = self.isUnsafeToMove()
        if unsafeToMove and not self.unsafeToMove:
            self.leadMotor.set_control(self.duty_cycle_request.with_output(0))
            self.setPositionGoal(self.getPosition())
        self.unsafeToMove = unsafeToMove
        # 2. do we need to find zero?
        if not self.zeroFound:
            self.findZero()
        # 3. report to the dashboard
        SmartDashboard.putString("elevState", self.getState())
        SmartDashboard.putNumber("elevGoal", self.getPositionGoal())
        SmartDashboard.putNumber("elevPosn", self.getPosition())

    def _getLeadMotorConfig(
            self,
            inverted: bool,
            limitSwitchNormallyOpen: bool,
            positionFactor: float,
            useAbsEncoder: bool,
    ) -> TalonFXConfiguration:
        config = TalonFXConfiguration()

        # Motor output configuration
        motor_out = MotorOutputConfigs()
        motor_out.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE if inverted else InvertedValue.CLOCKWISE_POSITIVE
        motor_out.neutral_mode = NeutralModeValue.BRAKE
        config.motor_output = motor_out

        # Limit switch configuration
        limit_config = HardwareLimitSwitchConfigs()
        limit_config.forward_limit_enable = True
        limit_config.reverse_limit_enable = True
        # Use correct limit switch type enums
        limit_config.forward_limit_type = ForwardLimitTypeValue.NORMALLY_OPEN if limitSwitchNormallyOpen else ForwardLimitTypeValue.NORMALLY_CLOSED
        limit_config.reverse_limit_type = ReverseLimitTypeValue.NORMALLY_OPEN if limitSwitchNormallyOpen else ReverseLimitTypeValue.NORMALLY_CLOSED
        config.hardware_limit_switch = limit_config

        # Feedback configuration
        feedback_config = FeedbackConfigs()
        feedback_config.sensor_to_mechanism = positionFactor
        config.feedback = feedback_config

        # PID configuration
        slot0 = Slot0Configs()
        slot0.k_p = ElevatorConstants.kP
        slot0.k_d = ElevatorConstants.kD
        slot0.k_s = ElevatorConstants.kStaticGain
        config.slot0 = slot0

        return config