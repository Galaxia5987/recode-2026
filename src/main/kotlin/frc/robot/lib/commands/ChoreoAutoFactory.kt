package frc.robot.lib.commands

import choreo.Choreo.TrajectoryCache
import choreo.Choreo.TrajectoryLogger
import choreo.auto.AutoFactory
import choreo.trajectory.Trajectory
import choreo.trajectory.TrajectorySample
import java.util.Optional
import java.util.function.Consumer
import java.util.function.Function
import java.util.function.Supplier
import org.wpilib.command2.Command as V2Command
import org.wpilib.command2.Subsystem
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Scheduler
import org.wpilib.driverstation.MatchState
import org.wpilib.math.geometry.Pose2d

/**
 * Runs Choreo's standalone commands on the Commands v3 scheduler.
 *
 * The v2 subsystem is only a requirement token. The returned commands reserve
 * [drive] in v3 and invoke the v2 lifecycle directly, without running a second
 * scheduler. Standalone commands and routine/trajectory wrappers all return v3
 * commands. As in upstream Choreo, trajectoryCmd does not reset odometry or run
 * event bindings; use a routine for trigger-based autonomous sequences.
 */
class ChoreoAutoFactory<Sample : TrajectorySample<Sample>>(
    poseSupplier: Supplier<Pose2d>,
    resetOdometry: Consumer<Pose2d>,
    controller: Consumer<Sample>,
    useAllianceFlipping: Boolean,
    private val drive: Mechanism,
    trajectoryLogger: TrajectoryLogger<Sample> = TrajectoryLogger { _, _ -> },
    internal val scheduler: Scheduler = Scheduler.getDefault(),
) {
    private val allianceFlipping = useAllianceFlipping
    private val bindings = linkedMapOf<String, Command>()
    private val subsystem = object : Subsystem {}
    private val factory =
        AutoFactory(
            poseSupplier,
            resetOdometry,
            controller,
            useAllianceFlipping,
            subsystem,
            trajectoryLogger,
        )

    fun newRoutine(name: String): ChoreoAutoRoutine<Sample> =
        ChoreoAutoRoutine(this, factory.newRoutine(name), name)

    fun trajectoryCmd(name: String): Command =
        adapt(factory.trajectoryCmd(name))

    fun trajectoryCmd(name: String, splitIndex: Int): Command =
        adapt(factory.trajectoryCmd(name, splitIndex))

    fun trajectoryCmd(
        name: String,
        transform:
            Function<
                ChoreoAutoTrajectory<Sample>,
                ChoreoAutoTrajectory<Sample>,
            >,
    ): Command =
        adapt(
            factory.trajectoryCmd(name) {
                transform.apply(ChoreoAutoTrajectory(this, it, null)).delegate
            }
        )

    fun trajectoryCmd(
        name: String,
        splitIndex: Int,
        transform:
            Function<
                ChoreoAutoTrajectory<Sample>,
                ChoreoAutoTrajectory<Sample>,
            >,
    ): Command =
        adapt(
            factory.trajectoryCmd(name, splitIndex) {
                transform.apply(ChoreoAutoTrajectory(this, it, null)).delegate
            }
        )

    fun trajectoryCmd(trajectory: Trajectory<Sample>): Command =
        adapt(factory.trajectoryCmd(trajectory))

    fun resetOdometry(name: String): Command =
        adapt(factory.resetOdometry(name))

    fun resetOdometry(name: String, splitIndex: Int): Command =
        adapt(factory.resetOdometry(name, splitIndex))

    fun resetOdometry(trajectory: Trajectory<Sample>): Command =
        adapt(factory.resetOdometry(trajectory))

    fun resetOdometry(pose: Supplier<Optional<Pose2d>>): Command =
        adapt(factory.resetOdometry(pose))

    fun resetOdometry(
        pose: Optional<Pose2d>,
        doFlipForAlliance: Boolean,
    ): Command = adapt(factory.resetOdometry(pose, doFlipForAlliance))

    fun bind(name: String, command: Command): ChoreoAutoFactory<Sample> =
        apply {
            bindings[name] = command
        }

    fun cache(): TrajectoryCache = factory.cache()

    internal fun allianceKnownOrIgnored(): Boolean =
        !allianceFlipping || MatchState.getAlliance().isPresent

    internal fun bindEvents(trajectory: ChoreoAutoTrajectory<Sample>) {
        bindings.forEach { (name, command) ->
            trajectory.active().and(trajectory.atTime(name)).onTrue(command)
        }
    }

    fun warmupCmd(): Command = adapt(factory.warmupCmd())

    internal fun adapt(command: V2Command): Command {
        require(command.requirements.all { it === subsystem }) {
            "Choreo command has an unmapped v2 subsystem requirement"
        }
        val lifecycle = V2CommandLifecycle(command)
        return Command.requiring(
                if (command.requirements.isEmpty()) emptySet() else setOf(drive)
            )
            .executing { coroutine ->
                try {
                    lifecycle.initialize()
                    while (!lifecycle.execute()) {
                        coroutine.yield()
                    }
                } catch (failure: Throwable) {
                    try {
                        lifecycle.end(true)
                    } catch (cleanupFailure: Throwable) {
                        failure.addSuppressed(cleanupFailure)
                    }
                    throw failure
                }
            }
            // Canceling a v3 coroutine does not unwind its suspended stack.
            .whenCanceled { lifecycle.end(true) }
            .named(command.name)
    }
}

/** Lifecycle state shared by the coroutine and its cancellation callback. */
internal class V2CommandLifecycle(private val command: V2Command) {
    private var active = false

    fun initialize() {
        check(!active) { "Command is already running" }
        active = true
        command.initialize()
    }

    /** Executes one scheduler tick and returns true on natural completion. */
    fun execute(): Boolean {
        check(active) { "Command has not been initialized" }
        command.execute()
        if (!command.isFinished) return false
        end(false)
        return true
    }

    fun end(interrupted: Boolean) {
        if (!active) return
        active = false
        command.end(interrupted)
    }
}
