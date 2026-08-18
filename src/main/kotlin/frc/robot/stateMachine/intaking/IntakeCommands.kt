package frc.robot.stateMachine.intaking

import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.spindexer.Spindexer
import org.wpilib.command3.Command

fun idle(): Command = command {
    +[Extender.close(), Roller.stop(), Spindexer.stop(), PreShooter.stop()]
}.named(IntakeStates.IDLE())


fun pumping(): Command = command {
    +[Extender.pump(), Roller.intake(), Spindexer.convey(), PreShooter.convey()]
}.named(IntakeStates.PUMPING())

fun intaking(): Command = command {
    +[Extender.open(), Roller.intake(), Spindexer.convey(), PreShooter.convey()]
}.named(IntakeStates.INTAKING())

fun outtaking(): Command = command {
    +[Extender.open(), Roller.intake(), Spindexer.convey(), PreShooter.convey()]
}.named(IntakeStates.OUTTAKING())
