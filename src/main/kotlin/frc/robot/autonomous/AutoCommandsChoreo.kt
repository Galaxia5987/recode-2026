package frc.robot.autonomous

import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import frc.robot.RobotContainer
import org.wpilib.command3.Command

fun line(): Command = resetAndFollow("line")

fun linewith180(): Command = resetAndFollow("linewith180")

fun digonal(): Command = resetAndFollow("digonal")

private fun resetAndFollow(name: String): Command =
    RobotContainer.autoFactory
        .resetOdometry(name)
        .andThen(RobotContainer.autoFactory.trajectoryCmd(name))
        .named("ResetAndFollow/$name")

fun pickupAuto(): AutoRoutine {
    val routine = RobotContainer.autoFactory
}
