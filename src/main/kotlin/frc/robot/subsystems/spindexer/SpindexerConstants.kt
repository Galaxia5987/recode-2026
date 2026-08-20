package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum
import org.team5987.annotation.command_enum.Priority
import org.wpilib.command3.Command
import org.wpilib.units.measure.AngularVelocity

const val PORT = 12

val SETPOINT_TOLERANCE = 0.1.rps
val GAINS =
    Gains(kP = 0.3, kS = 0.3, kV = 0.115)


val SIM_GAINS =
    Gains(
        kP = 1.0,
        kV = 1.2,
    )

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits = createCurrentLimits()
        Slot0 = GAINS.toSlotConfig()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.CounterClockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
    }

@CommandEnum
enum class SpindexerVelocities(
    val velocity: AngularVelocity,
    val priority: Priority,
) {
    STOP(0.rps, Priority(Command.LOWEST_PRIORITY)),
    CONVEY(30.rps, Priority(Command.HIGHEST_PRIORITY)),
}
