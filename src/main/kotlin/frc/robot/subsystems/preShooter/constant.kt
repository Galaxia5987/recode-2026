package frc.robot.subsystems.preShooter

import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum
import org.wpilib.units.measure.AngularVelocity

val SETPOINT_TOLERANCE = 0.1.rps

@CommandEnum
enum class PreShooerVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    CONVEY(10.rps),
}
