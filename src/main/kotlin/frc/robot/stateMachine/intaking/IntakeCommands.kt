package frc.robot.stateMachine.intaking

import frc.robot.lib.commands.command
import frc.robot.lib.commands.emptyCommand
import frc.robot.lib.commands.unaryPlus
import frc.robot.subsystems.intake.extender.Extender

fun idle()= emptyCommand()
fun pumping()= command {
    +[]
}
fun intaking()= emptyCommand()
fun outtaking()= emptyCommand()