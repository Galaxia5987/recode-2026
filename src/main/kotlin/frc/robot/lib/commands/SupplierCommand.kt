package frc.robot.lib.commands

import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SupplierCommandScope(
    val coroutine: Coroutine,
    val isContinuous: Boolean
) {
    /**
     * Executes the given block only if the command was invoked with a static target.
     * Use this to block execution (e.g., waiting for a setpoint) since supplier
     * commands are meant to run continuously without blocking.
     */
    inline fun whenOneShot(block: Coroutine.() -> Unit) {
        if (!isContinuous) {
            coroutine.block()
        }
    }
}

class SupplierCommand<T>(
    private val mechanism: Mechanism,
    private val commandName: String,
    private val action: SupplierCommandScope.(T) -> Unit
) {
    operator fun invoke(target: T): Command = mechanism {
        SupplierCommandScope(this, isContinuous = false).action(target)
    }.named(commandName)

    operator fun invoke(supplier: () -> T): Command = mechanism {
        val scope = SupplierCommandScope(this, isContinuous = true)
        while (true) {
            scope.action(supplier())
            yield()
        }
    }.named("${commandName}WithSupplier")
}

/**
 * Creates a command generator that supports both static targets and continuous suppliers.
 *
 * By defining the interaction once, this builder generates a class with overloaded
 * `invoke` operators. This allows the resulting property to be called as a standard command
 * or as a continuously looping supplier command.
 *
 * Use [SupplierCommandScope.whenOneShot] to define exit conditions (like waiting to reach a setpoint)
 * that should only apply to the one shot variation.
 *
 * @param name The base name for the generated commands. The supplier variant will automatically append "WithSupplier".
 * @param action The logic to apply the target value. Runs once for one shot commands, and continuously for supplier commands.
 *
 * Example:
 * ```kotlin
 * val setAngle = supplierCommand<Angle>("Subsystems/Turret/setAngle") { angle ->
 *     setpoint = constraintTurretLimit(angle)
 *     motor.setControl(positionVoltage.withPosition(setpoint))
 *
 *     whenOneShot {
 *         atSetpoint.waitUntil()
 *     }
 * }
 *
 * // Usage:
 * // val staticCmd = turret.setAngle(90.degrees)
 * // val dynamicCmd = turret.setAngle { SetpointManager.turretSetpoint }
 * ```
 */
fun <T> Mechanism.supplierCommand(
    name: String,
    action: SupplierCommandScope.(T) -> Unit
) = SupplierCommand(this, name, action)