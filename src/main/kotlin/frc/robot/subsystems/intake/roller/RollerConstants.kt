package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.MotorLogConfig

val MAIN_MOTOR_PORT = 10
val AUX_MOTOR_PORT = 30
val CLOCKWISE = 6.0.volts

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits =
            createCurrentLimits(
                supplyCurrentLimit = 25.amps,
                supplyCurrentPeakDifference = 5.amps,
            )
    }

val MOTOR_LOG_CONFIG =
    MotorLogConfig(
        position = false,
        statorCurrent = false,
        current = true,
        velocity = true,
        absoluteEncoder = false,
        voltage = true,
    )
