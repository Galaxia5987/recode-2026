package frc.robot.autonomous

import choreo.auto.AutoFactory
import frc.robot.RobotContainer
import frc.robot.drive
import frc.robot.lib.commands.ChoreoAutoFactory
import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.spindexer.Spindexer
import org.wpilib.command3.Command

fun ad() {
    val factory = AutoFactory(
        drive::getPose,
        drive::resetOdometry,
        drive::followTrajectory,
        true,
        drive,
    )
    val routine = RobotContainer.autoFactory.newRoutine("das")
    routine.cmd()
}

fun line(): Command = resetAndFollow("line")

fun linewith180(): Command = resetAndFollow("linewith180")

fun digonal(): Command = resetAndFollow("digonal")

private fun resetAndFollow(name: String): Command =
    RobotContainer.autoFactory
        .resetOdometry(name)
        .andThen(RobotContainer.autoFactory.trajectoryCmd(name))
        .named("ResetAndFollow/$name")

fun pickupAuto(): Command {
    val routine = RobotContainer.autoFactory.newRoutine("elinorPath")

    val pickupTraj = routine.trajectory("elinorPath")

    routine.active().onTrue(
        pickupTraj.resetOdometry()
            .andThen(pickupTraj.cmd()).named("auto")
    )

    pickupTraj.atTime("startIntaking").onTrue(Roller.intake())
    pickupTraj.done().onTrue(Roller.stop())

    return routine.cmd()
}

fun path1(): Command {
    val routine = RobotContainer.autoFactory.newRoutine("path1")
    val path1Traj = routine.trajectory("path1")

    routine.active().onTrue(
        path1Traj.resetOdometry()
            .andThen(path1Traj.cmd()).named("auto")
    )

    path1Traj.atTime("IntakeStart").onTrue(
        command {
            +[Roller.intake(), Spindexer.convey()]
        }.named("startIntake")

    )

    path1Traj.done().onTrue(
        command {
            +[Roller.stop(), Spindexer.stop()]
        }.named("stopIntake")
    )

    return routine.cmd()
}