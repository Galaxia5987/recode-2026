package frc.robot.lib.extensions

import java.util.function.BooleanSupplier
import org.wpilib.command3.Command
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Time

operator fun Trigger.not() = this.negate()

operator fun Trigger.get(time: Time) = this.debounce(time)

fun Trigger.debounce(seconds: Time) = this[seconds]

fun Trigger.and(vararg trigger: BooleanSupplier): Trigger =
    trigger.fold(this) { baseTrigger, trigger -> baseTrigger.and(trigger) }

fun Trigger.or(vararg trigger: BooleanSupplier): Trigger =
    trigger.fold(this) { baseTrigger, trigger -> baseTrigger.or(trigger) }

fun Trigger.onTrue(vararg commands: Command): Trigger =
    commands
        .apply {
            if (size == 1) {
                throw IllegalArgumentException(
                    "YOU CANNOT PASS ONE ARGUMENT TO ON_TRUE VARARG!!!!!!!!"
                )
            }
        }
        .fold(this) { baseTrigger, command -> baseTrigger.onTrue(command) }

fun Trigger.onFalse(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command -> baseTrigger.onFalse(command) }

fun Trigger.whileTrue(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command ->
        baseTrigger.whileTrue(command)
    }

fun Trigger.whileFalse(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command ->
        baseTrigger.whileFalse(command)
    }
