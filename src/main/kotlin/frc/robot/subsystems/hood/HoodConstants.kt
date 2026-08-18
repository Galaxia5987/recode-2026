package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.deg

val PORT = 0
val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutputConfigs().apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.Clockwise_Positive
        }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = 80.0
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = 40.0
            }
        Slot0 =
            Slot0Configs().apply {
                kP = 1.0
                kD = 0.0
            }
    }
val GEAR_RATIO = 1.0
val SIM_GAINS = Gains(kP = 1.43, kD = 0.36)
val ABSOLUTE_ENCODER_OFFSET = 0.deg

val TOLERANCE = 0.1.deg
