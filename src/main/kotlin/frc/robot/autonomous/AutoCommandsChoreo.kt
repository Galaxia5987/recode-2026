package frc.robot.autonomous

import frc.robot.RobotContainer
import org.wpilib.command3.Command

fun startToFuel2(): Command =
    resetAndFollow("StartToFuel2.traj")

fun elinorPath(): Command =
    resetAndFollow("elinorPath.traj")

fun joespath(): Command = resetAndFollow("joespath.traj")

private fun resetAndFollow(name: String): Command =
    RobotContainer.autoFactory.resetOdometry(name)
        .andThen(RobotContainer.autoFactory.trajectoryCmd(name))
        .named("ResetAndFollow/$name")
