package frc.robot.states.intake

import frc.robot.lib.extensions.onTrue

private val isIdle = IntakeState.IDLE.trigger.onTrue(idle())
private val isPumping = IntakeState.PUMPING.trigger.onTrue(idle())
private val isIntaking = IntakeState.INTAKING.trigger.onTrue(idle())
private val isOuttaking = IntakeState.OUTTAKING.trigger.onTrue(idle())
