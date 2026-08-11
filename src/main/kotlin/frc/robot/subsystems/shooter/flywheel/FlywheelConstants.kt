package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.mps
import frc.robot.lib.universal_motor.MotorLogConfig

const val MAIN_MOTOR_PORT = 2
const val FIRST_AUX_MOTOR_PORT = 3
const val SECOND_AUX_MOTOR_PORT = 4

const val GEAR_RATIO = 1.33
val REAL_GAINS = Gains(kP = 0.3, kS = 0.3, kV = 0.115)
val SIM_GAINS = Gains(kP = 1.0, kV = 0.12)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
        Slot0 = REAL_GAINS.toSlotConfig()

        CurrentLimits = createCurrentLimits(
            supplyCurrentLimit = 30.amps,
            supplyCurrentPeakDifference = 5.amps
        )
    }

val MOTOR_LOG_CONFIG = MotorLogConfig(
    position = false,
    statorCurrent = false,
    current = true,
    velocity = true,
    absoluteEncoder = false,
    voltage = true
)

val TOLERANCE = 1.deg_ps