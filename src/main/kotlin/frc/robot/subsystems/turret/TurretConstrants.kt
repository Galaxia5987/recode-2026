package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.lib.Gains
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rotations
import org.wpilib.units.measure.Angle

private val MAX_ANGLE: Angle = 270.0.deg
private val MIN_ANGLE: Angle = (-10.0).deg

val PORT = 1
val CONFIG = TalonFXConfiguration()
val GEAR_RATIO = 0.0
val SIM_GAINS = Gains()


// TODO: FIND PROBLEM
//val ENCODER_CONFIG =     CANcoderConfiguration().apply {
//    MagnetSensor.SensorDirection =
//        SensorDirectionValue.CounterClockwise_Positive
//    MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5
//    MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rotations]
//}

