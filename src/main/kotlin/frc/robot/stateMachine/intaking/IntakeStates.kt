package frc.robot.stateMachine.intaking

import frc.robot.lib.addStates
import org.wpilib.command3.StateMachine

enum class IntakeStates {
    IDLE,
    PUMPING,
    INTAKING,
    OUTTAKING;

    operator fun invoke() = name.lowercase()
}

val stateMachine = StateMachine("intake").apply {
    val idle = addState(idle())
    val pumping = addState(pumping())
    val intaking = addState(intaking())
    val outtaking = addState(outtaking())
    setInitialState(idle)

    idle.apply {
        switchTo(pumping).`when`()
    }

    pumping.apply {

    }
}


