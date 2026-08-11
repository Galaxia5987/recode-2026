package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.AngularVelocity

val SETPOINT_TOLERANCE = 0.1.rps
val GAINS =
    Gains(
        kP = 0.0,
        kD = 0.0,
        kV = 0.0,
    )

val SIM_GAINS =
    Gains(
        kP = 1.0,
        kV = 1.2,
    )

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits = CurrentLimitsConfigs()
        Slot0 = GAINS.toSlotConfig()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
    }

@CommandEnum
enum class SpindexerVelocities(val velocity: AngularVelocity) {
    STOP(0.rps),
    CONVEY(10.rps),
}
