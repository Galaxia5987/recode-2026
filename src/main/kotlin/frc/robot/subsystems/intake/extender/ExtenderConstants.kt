package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.volts
import org.wpilib.units.measure.AngularVelocity

const val PORT = 1

val SIM_GAINS = Gains(kP = 1.4, kD = 0.3)

val REAL_GAINS = Gains(kP = 3.5, kI = 2.0, kS = 2.0, kV = 2.5)

const val GEAR_RATIO = 1 / 0.5

val DIAMETER = 1.cm

val CLOSING_TIMEOUT = 10.sec
val CLOSING_MIN_VELOCITY: AngularVelocity = (-1).deg_ps

val CLOSE_POSITION_ANGLE = 12.15.rot

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Slot0 = REAL_GAINS.toSlotConfig()
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ReverseSoftLimitEnable = true
                ForwardSoftLimitThreshold = CLOSE_POSITION_ANGLE[rot]
                ReverseSoftLimitThreshold = 0.0
            }
        CurrentLimits = createCurrentLimits(25.amps, 5.amps)
    }

enum class ExtenderState {
    OPEN,
    CLOSE,
    IDLE,
}

val TOLERANCE = 3.cm

val CLOSING_VOLTAGE = (-1).volts

val OPEN_POSITION = 0.304.meters
val OPEN_POSITION_ANGLE = OPEN_POSITION.toAngle(DIAMETER, GEAR_RATIO)
