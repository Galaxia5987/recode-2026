package frc.robot.autonomous

import frc.robot.RobotContainer
import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.intake.roller.Roller
import org.wpilib.command3.Command

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
