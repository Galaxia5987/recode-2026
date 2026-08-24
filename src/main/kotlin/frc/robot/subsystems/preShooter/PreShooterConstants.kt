package frc.robot.subsystems.preShooter

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.MotorLogConfig
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.AngularVelocity

val SETPOINT_TOLERANCE = 0.1.rps
const val PORT = 13
const val GEAR_RATIO = 1.0
val REAL_GAINS = Gains(kP = 0.3, kV = 0.15)
val SIM_GAINS = Gains()

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 = REAL_GAINS.toSlotConfig()
    }

val LOG_CONFIG =
    MotorLogConfig(
        position = false,
        statorCurrent = false,
        current = true,
        velocity = true,
        absoluteEncoder = false,
        voltage = true,
        controlRequest = false,
    )

@CommandEnum
enum class PreShooterVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    CONVEY(30.rps),
}
