package frc.robot.lib.commands

import choreo.auto.AutoRoutine
import choreo.trajectory.Trajectory
import choreo.trajectory.TrajectorySample
import java.util.function.BooleanSupplier
import org.wpilib.command3.Command
import org.wpilib.driverstation.RobotState
import org.wpilib.event.EventLoop

/** A Choreo event loop whose bindings and children run on the v3 scheduler. */
class ChoreoAutoRoutine<Sample : TrajectorySample<Sample>>
internal constructor(
    internal val factory: ChoreoAutoFactory<Sample>,
    private val delegate: AutoRoutine,
    private val name: String,
) {
    private val boundCommands = mutableListOf<Command>()
    private val pendingSpawns = mutableListOf<Command>()
    private val spawnedCommands = mutableSetOf<Command>()
    private var killed = false

    fun active(): ChoreoTrigger = trigger(delegate.active())

    fun idle(): ChoreoTrigger = trigger(delegate.idle())

    fun observe(condition: BooleanSupplier): ChoreoTrigger = trigger(condition)

    fun loop(): EventLoop = delegate.loop()

    fun poll() {
        if (killed) return
        delegate.poll()
        val pending = pendingSpawns.toList()
        pendingSpawns.clear()
        pending.forEach {
            spawnedCommands.add(it)
            factory.scheduler.schedule(it)
        }
        spawnedCommands.removeAll {
            !factory.scheduler.isScheduledOrRunning(it)
        }
    }

    fun reset() {
        (boundCommands + spawnedCommands).forEach {
            factory.scheduler.cancel(it)
        }
        pendingSpawns.clear()
        spawnedCommands.clear()
        delegate.reset()
    }

    /** Cancels this routine's commands, leaving unrelated v3 commands alone. */
    fun kill() {
        killed = true
        reset()
    }

    fun trajectory(name: String): ChoreoAutoTrajectory<Sample> =
        wrap(delegate.trajectory(name))

    fun trajectory(
        name: String,
        splitIndex: Int,
    ): ChoreoAutoTrajectory<Sample> =
        wrap(delegate.trajectory(name, splitIndex))

    fun trajectory(
        trajectory: Trajectory<Sample>
    ): ChoreoAutoTrajectory<Sample> = wrap(delegate.trajectory(trajectory))

    internal fun wrap(
        trajectory: choreo.auto.AutoTrajectory
    ): ChoreoAutoTrajectory<Sample> =
        ChoreoAutoTrajectory(factory, trajectory, this).also {
            factory.bindEvents(it)
        }

    fun anyDone(
        first: ChoreoAutoTrajectory<Sample>,
        vararg rest: ChoreoAutoTrajectory<Sample>,
    ): ChoreoTrigger = anyDoneDelayed(0, first, *rest)

    fun anyDoneDelayed(
        cyclesToDelay: Int,
        first: ChoreoAutoTrajectory<Sample>,
        vararg rest: ChoreoAutoTrajectory<Sample>,
    ): ChoreoTrigger =
        trigger(
            delegate.anyDoneDelayed(
                cyclesToDelay,
                first.delegate,
                *rest.map { it.delegate }.toTypedArray(),
            )
        )

    fun anyActive(
        first: ChoreoAutoTrajectory<Sample>,
        vararg rest: ChoreoAutoTrajectory<Sample>,
    ): ChoreoTrigger =
        trigger(
            delegate.anyActive(
                first.delegate,
                *rest.map { it.delegate }.toTypedArray(),
            )
        )

    fun allInactive(
        first: ChoreoAutoTrajectory<Sample>,
        vararg rest: ChoreoAutoTrajectory<Sample>,
    ): ChoreoTrigger =
        trigger(
            delegate.allInactive(
                first.delegate,
                *rest.map { it.delegate }.toTypedArray(),
            )
        )

    fun cmd(): Command = cmd { false }

    fun cmd(finishCondition: BooleanSupplier): Command =
        Command.noRequirements { coroutine ->
                reset()
                try {
                    if (!factory.allianceKnownOrIgnored()) kill()
                    while (
                        !killed &&
                            !RobotState.isDisabled() &&
                            !finishCondition.asBoolean
                    ) {
                        poll()
                        coroutine.yield()
                    }
                } finally {
                    reset()
                }
            }
            .whenCanceled { reset() }
            .named(name)

    internal fun trigger(condition: BooleanSupplier): ChoreoTrigger =
        ChoreoTrigger(this, condition)

    internal fun scoped(command: Command): Command =
        Command.noRequirements { coroutine -> coroutine.await(command) }
            .named("$name/Binding/${command.name()}")
            .also { track(it) }

    internal fun track(command: Command) {
        boundCommands.add(command)
    }

    internal fun spawn(command: Command) {
        check(!killed) { "Cannot spawn a command on a killed routine" }
        pendingSpawns.add(command)
    }
}
