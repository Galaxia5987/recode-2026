package frc.robot.states.intake

import frc.robot.lib.commands.command
import frc.robot.lib.commands.invoke
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger

enum class IntakeState {
    IDLE,
    PUMPING,
    INTAKING,
    OUTTAKING;

    val trigger = Trigger { state == this }

    fun set(): Command =
        command {
                state = this@IntakeState
            }
            .named("IntakeStates/Set")
}

private var state: IntakeState = IntakeState.IDLE
