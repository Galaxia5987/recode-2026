package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.configs.TalonFXConfiguration
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.mps

const val PORT = 2

val CONFIG = TalonFXConfiguration()

val TOLERANCE = 1.deg_ps