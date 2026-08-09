package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.TalonFXConfiguration
import frc.robot.lib.Gains
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.volts
import org.wpilib.units.measure.AngularVelocity

val PORT = 1

val SIM_GAINS = Gains()
val REAL_GAINS = Gains()

val GEAR_RATIO = 1 / 0.5

val DIAMETER = 1.0.cm

val CLOSING_MIN_VELOCITY: AngularVelocity = 1.0.deg_ps

val CONFIG = TalonFXConfiguration()

enum class ExtenderState {
    OPENING, CLOSING, IDLE
}

val CLOSING_VOLTAGE = 1.volts