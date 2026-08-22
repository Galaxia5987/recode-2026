package frc.robot.states.intake

import frc.robot.lib.commands.command
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller
import org.wpilib.command3.Command

fun idle(): Command =
    command {
            +[Roller.stop(), Extender.close()]
        }
        .named("IntakeCommands/Idle")

fun pumping(): Command =
    command {
            +[Roller.intake(), Extender.pump()]
        }
        .named("IntakeCommands/Pumping")

fun intaking(): Command =
    command {
            +[Roller.intake(), Extender.open()]
        }
        .named("IntakeCommands/Intaking")

fun outtaking(): Command =
    command {
            +[Roller.outtake(), Extender.open()]
        }
        .named("IntakeCommands/Outtaking")
