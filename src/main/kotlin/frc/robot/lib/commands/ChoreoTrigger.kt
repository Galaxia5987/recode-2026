package frc.robot.lib.commands

import java.util.function.BooleanSupplier
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger

/**
 * Routine-local v3 trigger; bound commands are canceled when the routine ends.
 */
class ChoreoTrigger
internal constructor(
    private val routine: ChoreoAutoRoutine<*>,
    private val condition: BooleanSupplier,
) : BooleanSupplier {
    private val trigger =
        Trigger(routine.factory.scheduler, routine.loop(), condition)

    override fun getAsBoolean(): Boolean = condition.asBoolean

    fun onTrue(command: Command): ChoreoTrigger = apply {
        trigger.onTrue(routine.scoped(command))
    }

    fun onFalse(command: Command): ChoreoTrigger = apply {
        trigger.onFalse(routine.scoped(command))
    }

    fun whileTrue(command: Command): ChoreoTrigger = apply {
        trigger.whileTrue(routine.scoped(command))
    }

    fun whileFalse(command: Command): ChoreoTrigger = apply {
        trigger.whileFalse(routine.scoped(command))
    }

    fun toggleOnTrue(command: Command): ChoreoTrigger = apply {
        trigger.toggleOnTrue(routine.scoped(command))
    }

    fun toggleOnFalse(command: Command): ChoreoTrigger = apply {
        trigger.toggleOnFalse(routine.scoped(command))
    }

    fun and(other: BooleanSupplier): ChoreoTrigger = routine.trigger {
        asBoolean && other.asBoolean
    }

    fun or(other: BooleanSupplier): ChoreoTrigger = routine.trigger {
        asBoolean || other.asBoolean
    }

    fun negate(): ChoreoTrigger = routine.trigger { !asBoolean }
}
