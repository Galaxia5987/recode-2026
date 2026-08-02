package frc.robot.lib.commands

import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.command3.NeedsNameBuilderStage

fun emptyCommand(): Command =
    Command.noRequirements(Coroutine::park)
        .withPriority(Command.LOWEST_PRIORITY)
        .named("EmptyCommand")

context(coroutine: Coroutine)
operator fun Command.unaryPlus() = coroutine.await(this)

context(coroutine: Coroutine)
operator fun List<Command>.unaryPlus() = coroutine.awaitAll(*(this.toTypedArray()))

inline fun noRequirements(crossinline block: Coroutine.() -> Unit): NeedsNameBuilderStage {
    return Command.noRequirements { coroutine ->
        coroutine.block()
    }
}

inline operator fun Mechanism.invoke(crossinline block: Coroutine.() -> Unit): NeedsNameBuilderStage {
    return this.run { coroutine ->
        coroutine.block()
    }
}

operator fun Coroutine.not() = yield()