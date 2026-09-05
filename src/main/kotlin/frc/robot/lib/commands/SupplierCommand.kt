package frc.robot.lib.commands

import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism

class SupplierCommandScope(
    val coroutine: Coroutine,
    val isContinuous: Boolean
) {
    inline fun onCompletion(block: Coroutine.() -> Unit) {
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

fun <T> Mechanism.supplierCommand(
    name: String,
    action: SupplierCommandScope.(T) -> Unit
) = SupplierCommand(this, name, action)