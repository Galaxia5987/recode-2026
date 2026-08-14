package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.universal_motor.MotorLogConfig

val MAIN_MOTOR_PORT = 1
val AUX_MOTOR_PORT = 2
val GEAR_RATIO = 1.33



val MOTOR_CONFIG =TalonFXConfiguration().apply {
    MotorOutput =
        MotorOutputConfigs().apply{
            Inverted = InvertedValue.Clockwise_Positive
        }
    CurrentLimits =
        createCurrentLimits(
            supplyCurrentLimit = 35.amps,
            supplyCurrentPeakDifference = 6.amps,
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