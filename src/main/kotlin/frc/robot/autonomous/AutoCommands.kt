package frc.robot.autonomous

import choreo.auto.AutoFactory
import frc.robot.drive
import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import org.wpilib.command3.Command

val autoFactory =
    AutoFactory(
        drive::getPose,
        drive::resetOdometry,
        drive::followTrajectory,
        true,
        drive,
    )

fun line(): Command = resetAndFollow("line")

fun linewith180(): Command = resetAndFollow("linewith180")

fun digonal(): Command = resetAndFollow("digonal")

private fun resetAndFollow(name: String): Command {
    val trajectory = autoFactory.trajectory(name)
    return command {
            +trajectory.resetOdometry()
            +trajectory.cmd()
        }
        .named("resetAndFollow/$name")
}
