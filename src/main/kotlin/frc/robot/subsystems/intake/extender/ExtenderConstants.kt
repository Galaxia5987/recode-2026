package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.TalonFXConfiguration
import frc.robot.lib.Gains
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.volts
import org.wpilib.units.measure.AngularVelocity

val PORT = 1

val SIM_GAINS = Gains(kP = 1.4, kD = 0.3)

val REAL_GAINS = Gains(kP = 3.5, kI = 2.0, kS = 2.0, kV = 2.5)

val GEAR_RATIO = 1 / 0.5

val DIAMETER = 1.cm

val CLOSING_MIN_START_VELOCITY = -1.deg_ps
val CLOSING_MIN_VELOCITY: AngularVelocity = -1.deg_ps

val CONFIG = TalonFXConfiguration()

enum class ExtenderState {
    OPEN, CLOSE, IDLE
}

val TOLERANCE = 3.cm

val CLOSING_VOLTAGE = -1.volts

val OPEN_POSITION = 0.304.meters