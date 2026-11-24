// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.state_machine

import frc.robot.lib.commands.schedule
import java.util.function.BooleanSupplier
import kotlin.reflect.KClass
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Command.noRequirements
import org.wpilib.command3.Coroutine
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

/**
 * A declarative state machine that can be used to implement complex command
 * routines. This combines the execution logic of the WPILib StateMachine with a
 * native Kotlin DSL.
 */
class StateMachine<E : Enum<E>>(
    private val name: String,
    private val log: Boolean = true,
) : Command {

    private val stateMap = mutableMapOf<E, State>()
    private var initialState: State? = null
    private val stateChangeCallbacks = mutableListOf<(newState: E) -> Unit>()

    /** The state that is currently running, or null while not scheduled. */
    private var activeState: State? = null

    /** Set by [forceTransition], consumed on the next loop iteration. */
    private var forcedState: State? = null

    /** True from the start of [run] until the machine is cancelled. */
    private var running = false

    /** The state marked with [initial], or null if none was marked yet. */
    val initialStateValue: E?
        get() = initialState?.enumVal

    override fun name(): String = name

    override fun requirements(): Set<Mechanism> = emptySet()

    override fun run(coroutine: Coroutine) {
        var state =
            initialState
                ?: error(
                    "$name does not have an initial state. Use .initial() to provide one."
                )
        forcedState = null
        running = true

        while (true) {
            enter(state, coroutine)
            val next = awaitNextState(state, coroutine)
            exit(state, coroutine)
            // Always hand control back to the scheduler between states, so a
            // cycle of states that finish instantly cannot stall the robot
            // loop.
            coroutine.yield()
            // A forced transition requested during the yield takes precedence.
            state = forcedState?.also { forcedState = null } ?: next
        }
    }

    private fun enter(state: State, coroutine: Coroutine) {
        activeState = state
        coroutine.fork(state.command)
        state.enterCallbacks.forEach { it.run() }
    }

    private fun exit(state: State, coroutine: Coroutine) {
        state.exitCallbacks.forEach { it.run() }
        coroutine.scheduler().cancel(state.command)
        activeState = null
    }

    /** Blocks until the current state should be left, and returns the next. */
    private fun awaitNextState(state: State, coroutine: Coroutine): State {
        while (true) {
            forcedState?.let {
                forcedState = null
                return it
            }

            state.transitions
                .firstOrNull { it.shouldTransition() }
                ?.let {
                    return it.target()
                }

            if (!coroutine.scheduler().isRunning(state.command)) {
                state.nextStateOnComplete()?.let {
                    return it
                }
            }

            coroutine.yield()
        }
    }

    // Exit callbacks often stop motors, so they must also run when the whole
    // machine is cancelled, not only on transitions.
    override fun onCancel() {
        activeState?.exitCallbacks?.forEach { it.run() }
        activeState = null
        forcedState = null
        running = false
    }

    /**
     * Leaves the current state and enters [target] on the next loop, running
     * the usual exit and enter callbacks. Has no effect while not scheduled.
     */
    fun forceTransition(target: E) {
        if (running) forcedState = getState(target)
    }

    // --------------------------------------------------------
    // Internal Components
    // --------------------------------------------------------

    inner class State(val enumVal: E, val command: Command) {
        val transitions = mutableListOf<Transition>()
        val completions = mutableListOf<Transition>()
        var defaultNextState: (() -> State)? = null
        val enterCallbacks = mutableListOf<Runnable>()
        val exitCallbacks = mutableListOf<Runnable>()

        fun addCompletion(condition: BooleanSupplier, next: () -> State) {
            completions.removeIf { it.condition === condition }
            completions.add(Transition(next, condition))
        }

        fun nextStateOnComplete(): State? =
            completions.firstOrNull { it.shouldTransition() }?.target()
                ?: defaultNextState?.invoke()

        fun onEnter(callback: Runnable) {
            enterCallbacks.add(callback)
        }

        fun onExit(callback: Runnable) {
            exitCallbacks.add(callback)
        }
    }

    inner class Transition(
        val target: () -> State,
        val condition: BooleanSupplier,
    ) {
        fun shouldTransition(): Boolean = condition.asBoolean
    }

    // --------------------------------------------------------
    // Kotlin DSL
    // --------------------------------------------------------

    /** Adds a callback that runs every time a state is entered. */
    fun onStateChange(callback: (newState: E) -> Unit) {
        stateChangeCallbacks.add(callback)
    }

    operator fun E.invoke(command: Command): E {
        require(!stateMap.containsKey(this)) {
            "State $this is already defined."
        }

        val stateEnum = this
        val state = State(stateEnum, command)
        if (log) {
            // Keyed by machine name, so two machines sharing an enum don't
            // overwrite each other's log entry.
            val logPath = "States/${this@StateMachine.name}/state"
            state.onEnter { Logger.recordOutput(logPath, stateEnum.name) }
        }
        state.onEnter { stateChangeCallbacks.forEach { it(stateEnum) } }
        stateMap[stateEnum] = state
        return stateEnum
    }

    operator fun E.invoke(block: Coroutine.() -> Unit): E {
        this(noRequirements { coroutine -> coroutine.block() }.named(name))
        return this
    }

    fun E.initial() {
        initialState = getState(this)
    }

    /** Runs [callback] every time this state is entered. */
    infix fun E.onEnter(callback: () -> Unit) = getState(this).onEnter(callback)

    /**
     * Runs [callback] every time this state is left, including when the whole
     * machine is cancelled. Use it to stop anything the state started.
     */
    infix fun E.onExit(callback: () -> Unit) = getState(this).onExit(callback)

    private fun getState(enumVal: E): State {
        return stateMap[enumVal]
            ?: error(
                "State $enumVal was used in a transition but never defined."
            )
    }

    enum class TransitionType {
        ON,
        COMPLETE_AND,
    }

    inner class TransitionCondition(
        val source: E,
        val condition: Trigger,
        val transitionType: TransitionType,
    )

    inner class MultiTransitionCondition(
        val sources: List<E>,
        val condition: Trigger,
    )

    inner class CompleteTransitionWrapper(val source: E)

    infix fun E.on(trigger: Trigger): TransitionCondition =
        TransitionCondition(this, trigger, TransitionType.ON)

    infix fun E.on(condition: () -> Boolean): TransitionCondition =
        on(Trigger(condition))

    infix fun E.completeAnd(trigger: Trigger): TransitionCondition =
        TransitionCondition(this, trigger, TransitionType.COMPLETE_AND)

    infix fun E.completeAnd(condition: () -> Boolean): TransitionCondition =
        completeAnd(Trigger(condition))

    infix fun TransitionCondition.switchTo(target: E) {
        val sourceState = getState(this.source)
        val targetSupplier = { getState(target) }

        when (transitionType) {
            TransitionType.ON ->
                sourceState.transitions.add(
                    Transition(targetSupplier, condition)
                )
            TransitionType.COMPLETE_AND ->
                sourceState.addCompletion(condition, targetSupplier)
        }
    }

    infix fun List<E>.on(trigger: Trigger): MultiTransitionCondition =
        MultiTransitionCondition(this, trigger)

    infix fun List<E>.on(condition: () -> Boolean): MultiTransitionCondition =
        on(Trigger(condition))

    infix fun MultiTransitionCondition.switchTo(target: E) {
        val targetSupplier = { getState(target) }
        this.sources.forEach { sourceEnum ->
            getState(sourceEnum)
                .transitions
                .add(Transition(targetSupplier, condition))
        }
    }

    val E.onComplete: CompleteTransitionWrapper
        get() = CompleteTransitionWrapper(this)

    infix fun CompleteTransitionWrapper.switchTo(target: E) {
        getState(this.source).defaultNextState = { getState(target) }
    }
}

// --------------------------------------------------------
// Utilities & Extensions
// --------------------------------------------------------

inline fun <reified E : Enum<E>> allOf(): List<E> = enumValues<E>().toList()

fun <E : Enum<E>> buildStateMachine(
    name: String,
    init: StateMachine<E>.() -> Unit,
): StateMachine<E> {
    return StateMachine<E>(name).apply(init)
}

abstract class StateMachineCompanion<T : Enum<T>>(
    private val stateClass: KClass<T>
) {
    private var currentState: T? = null

    /**
     * The state the machine is in, or its initial state before it has been
     * registered.
     */
    val state: T
        get() =
            currentState
                ?: stateMachine.initialStateValue
                ?: error(
                    "${stateClass.simpleName} does not have an initial state."
                )

    protected abstract val states: StateMachine<T>.() -> Unit

    protected fun makeStates(
        init: StateMachine<T>.() -> Unit
    ): StateMachine<T>.() -> Unit = init

    fun trigger(state: T) = Trigger { state == this.state }

    /**
     * Moves the running machine to [state]; see [StateMachine.forceTransition].
     */
    fun forceTransition(state: T) = stateMachine.forceTransition(state)

    val stateMachine by lazy {
        buildStateMachine<T>(stateClass.simpleName!!) {
            // Registered first so `state` is already updated when user
            // onStateChange callbacks run.
            onStateChange { currentState = it }
            states()
        }
    }

    fun register() = stateMachine.schedule()
}
