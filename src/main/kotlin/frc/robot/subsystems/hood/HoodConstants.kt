package frc.robot.subsystems.hood

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.deg

val PORT = 0
val CANBUS = systemcore(1)
val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
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
val ABSOLUTE_ENCODER_OFFSET = 0.0

val TOLERANCE = 0.1.deg

val ENCODER_ID = 0
val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5
    }

