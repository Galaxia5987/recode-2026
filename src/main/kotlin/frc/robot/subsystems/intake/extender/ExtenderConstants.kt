package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.TalonFXConfiguration
import frc.robot.lib.Gains
import frc.robot.lib.extensions.cm

val PORT = 1

val SIM_GAINS = Gains()
val REAL_GAINS = Gains()

val GEAR_RATIO = 1 / 0.5

val DIAMETER = 1.0.cm

val CONFIG = TalonFXConfiguration()

enum class ExtenderState {
    OPENING, CLOSING, IDLE
}