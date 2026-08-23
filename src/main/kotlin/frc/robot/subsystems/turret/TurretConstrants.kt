package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot

val PORT = 1
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 0.5, kD = 0.075)
const val RATIO = 1.0
val ENCODER_ID = 0

val FORWARD_LIMIT = 360.deg // I made it a separate parameter for ease of use
val REVERSE_LIMIT = 0.0.rot

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5
        MagnetSensor.MagnetOffset = 0.0
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 = REAL_GAINS.toSlotConfig()
        // This config forces the motor to not move beyond these limits
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold =
                    FORWARD_LIMIT[rot] // The thresholds work in rotations.
                ReverseSoftLimitEnable = true
                ReverseSoftLimitThreshold = REVERSE_LIMIT[rot]
            }

        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }
    }

val TOLERANCE = 1.deg
