from commands2 import Subsystem
from wpilib import SmartDashboard
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import (
    HardwareLimitSwitchConfigs,
    MotorOutputConfigs
)
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import (
    ForwardLimitTypeValue,
    InvertedValue,
    NeutralModeValue
)


class Intake(Subsystem):
    def __init__(self,
                 leaderCanID,
                 leaderInverted=True,
                 followerCanID=None,
                 followerInverted=False,
                 rangeFinder=None,
                 rangeToGamepiece=None) -> None:
        """
        :param leaderCanID: CAN ID of the leader motor (or of your only motor)
        :param leaderInverted: is the leader motor inverted?
        :param followerCanID: CAN ID of the follower motor, if we have it
        :param followerInverted: is follower motor inverted?
        :param rangeFinder: do we have a rangefinder to sense gamepieces?
        :param rangeToGamepiece: any range closer than this will count as "gamepiece in"
        """
        super().__init__()

        # 0. state
        self.limitSwitchSensingGamepiece = False
        self.rangeFinderSensingGamepiece = False
        self.sensingGamepiece = False

        self.desiredSpeedL = 0
        self.desiredSpeedF = 0
        self.stopIfSensingGamepiece = False

        # Create motor configuration
        config = self._getMotorConfig(
            inverted=leaderInverted,
            limitSwitchNormallyOpen=True
        )

        # 1. Setup the leader motor
        self.motor = TalonFX(leaderCanID)
        self.motor.configurator.apply(config)

        # Create control request for duty cycle output
        self.duty_cycle_request = DutyCycleOut(0)

        # 2. Setup follower motor if provided
        self.followerMotor = None
        if followerCanID is not None:
            follower_config = self._getMotorConfig(
                inverted=followerInverted,
                limitSwitchNormallyOpen=True
            )
            self.followerMotor = TalonFX(followerCanID)
            self.followerMotor.configurator.apply(follower_config)

        # 3. If we have a rangefinder, set it up
        self.rangeFinder = rangeFinder
        self.rangeToGamepiece = rangeToGamepiece

    def _getMotorConfig(self, inverted: bool, limitSwitchNormallyOpen: bool) -> TalonFXConfiguration:
        config = TalonFXConfiguration()

        # Motor output configuration
        motor_out = MotorOutputConfigs()
        motor_out.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE if inverted else InvertedValue.CLOCKWISE_POSITIVE
        motor_out.neutral_mode = NeutralModeValue.BRAKE
        config.motor_output = motor_out

        # Limit switch configuration
        limit_config = HardwareLimitSwitchConfigs()
        limit_config.forward_limit_enable = True
        limit_config.forward_limit_type = ForwardLimitTypeValue.NORMALLY_OPEN if limitSwitchNormallyOpen else ForwardLimitTypeValue.NORMALLY_CLOSED
        config.hardware_limit_switch = limit_config

        return config

    def enableLimitSwitch(self):
        self.stopIfSensingGamepiece = True

    def disableLimitSwitch(self):
        self.stopIfSensingGamepiece = False

    def isGamepieceInside(self) -> bool:
        return self.sensingGamepiece

    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()

    def periodic(self):
        # 1. Check if limit switch or rangefinder is sensing the gamepiece
        self.limitSwitchSensingGamepiece = self.motor.get_forward_limit().value
        SmartDashboard.putBoolean("intakeSwitchPressed", self.limitSwitchSensingGamepiece)

        if self.rangeFinder is not None:
            range = self.rangeFinder.getRange()
            SmartDashboard.putNumber("intakeRangeToGamepiece", range)
            self.rangeFinderSensingGamepiece = range != 0 and range <= self.rangeToGamepiece

        # 2. We say we are sensing that gamepiece if either limit switch or rangefinder is sensing it
        self.sensingGamepiece = self.limitSwitchSensingGamepiece or self.rangeFinderSensingGamepiece
        SmartDashboard.putBoolean("intakeFull", self.sensingGamepiece)
        SmartDashboard.putNumber("intakeDesiredSpeedL", self.desiredSpeedL)
        if self.followerMotor is not None:
            SmartDashboard.putNumber("intakeDesiredSpeedF", self.desiredSpeedF)

        # 3. If we are sensing the gamepiece, maybe stop the motor (otherwise, spin it)
        speedL, speedF = self.desiredSpeedL, self.desiredSpeedF
        if self.sensingGamepiece and self.stopIfSensingGamepiece:
            speedL, speedF = 0.0, 0.0

        self.motor.set_control(self.duty_cycle_request.with_output(speedL))
        SmartDashboard.putNumber("intakeSpeedL", speedL)
        if self.followerMotor is not None:
            self.followerMotor.set_control(self.duty_cycle_request.with_output(speedF))
            SmartDashboard.putNumber("intakeSpeedF", speedF)

    def intakeGamepiece(self, speed=0.25, speedF=None):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(speed, speedF)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self, speed=1.0, speedF=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, speedF)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25, speedF=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        if speedF is None:
            speedF = speed
        self._setSpeed(-speed, -speedF)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25, speedF=None):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, speedF)

    def stop(self):
        self.desiredSpeedL, self.desiredSpeedF = 0.0, 0.0
        self.motor.set_control(self.duty_cycle_request.with_output(0))
        if self.followerMotor is not None:
            self.followerMotor.set_control(self.duty_cycle_request.with_output(0))
        print("Intake::stop")

    def _setSpeed(self, speedL, speedF=None):
        self.desiredSpeedL = speedL
        if speedF is not None:
            self.desiredSpeedF = speedF
        else:
            self.desiredSpeedF = speedL