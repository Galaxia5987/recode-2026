package frc.robot

import org.wpilib.command3.Command
import org.wpilib.command3.Command.noRequirements
import org.wpilib.command3.Coroutine
import org.wpilib.command3.StateMachine
import org.wpilib.command3.Trigger
import kotlin.enums.EnumEntries

interface StateEnum {
    var state: StateMachine.State?
}

enum class IntakeStates : StateEnum {
    INTAKING,
    PUMPING,
    CLOSED;

    override var state: StateMachine.State? = null
}

fun <T> EnumEntries<T>.all(): List<StateMachine.State?> where T : Enum<T>, T : StateEnum = map { (this as StateEnum).state }

context(stateMachine: StateMachine)
operator fun <T> T.invoke(command: Command): T where T : Enum<T>, T : StateEnum {
    this.state = stateMachine.addState(command)
    return this
}

context(stateMachine: StateMachine)
inline operator fun <T> T.invoke(crossinline block: Coroutine.() -> Unit): T where T : Enum<T>, T : StateEnum =
    this(noRequirements { coroutine -> coroutine.block() }.named(name))

context(stateMachine: StateMachine)
fun <T> T.initial(): T where T : Enum<T>, T : StateEnum {
    stateMachine.setInitialState(state)
    return this
}

class TransitionCondition(
    val source: StateEnum,
    val condition: Trigger
)

infix fun StateEnum.on(trigger: Trigger): TransitionCondition {
    return TransitionCondition(this, trigger)
}

infix fun StateEnum.on(condition: () -> Boolean): TransitionCondition =
    on(Trigger(condition))

context(stateMachine: StateMachine)
infix fun TransitionCondition.switchTo(target: StateEnum) {
    this.source.state!!.switchTo(target.state!!).`when`(this.condition)
}

class MultiTransitionCondition(
    val sources: Array<out StateEnum>,
    val condition: Trigger
)

fun anyOf(vararg states: StateEnum): Array<out StateEnum> = states

infix fun Array<out StateEnum>.on(trigger: Trigger): MultiTransitionCondition {
    return MultiTransitionCondition(this, trigger)
}

infix fun Array<out StateEnum>.on(condition: () -> Boolean): MultiTransitionCondition =
    on(Trigger(condition))


context(stateMachine: StateMachine)
infix fun MultiTransitionCondition.switchTo(target: StateEnum) {
    val unwrappedStates = this.sources.map { it.state!! }.toTypedArray()
    stateMachine.switchFromAny(*unwrappedStates).to(target.state!!).`when`(this.condition)
}

class CompleteTransitionWrapper(val source: StateEnum)

val StateEnum.onComplete: CompleteTransitionWrapper
    get() = CompleteTransitionWrapper(this)

context(stateMachine: StateMachine)
infix fun CompleteTransitionWrapper.switchTo(target: StateEnum) {
    this.source.state!!.switchTo(target.state!!).whenComplete()
}

inline fun <reified T> allOf(): Array<out StateEnum> where T : Enum<T>, T : StateEnum {
    return enumValues<T>()
}


val stateMachine = StateMachine("State Machine").apply {
    IntakeStates.INTAKING {

    }.initial()

    IntakeStates.INTAKING on { true } switchTo IntakeStates.PUMPING

    allOf<IntakeStates>() on { true }
}