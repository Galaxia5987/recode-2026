package frc.robot.subsystems.hood

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg

const val PORT = 0
val CANBUS = systemcore(0)

val SIM_GAINS = Gains(kP = 1.43, kD = 0.36)
val REAL_GAINS = Gains(kP = 1.0)

const val GEAR_RATIO = 46.77199935913086
const val ENCODER_GEAR_RATIO = 2.0
const val ENCODER_ID = 0

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits(15.amps, 5.amps)
        Slot0 =
            REAL_GAINS.toSlotConfig()
                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign
                )
        Feedback =
            FeedbackConfigs().apply {
                RotorToSensorRatio = GEAR_RATIO
                SensorToMechanismRatio = ENCODER_GEAR_RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }
    }
val ABSOLUTE_ENCODER_OFFSET = 0.0

val TOLERANCE = 0.5.deg

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5
    }
