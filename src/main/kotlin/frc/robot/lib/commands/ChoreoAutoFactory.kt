package frc.robot.lib.commands

import choreo.auto.AutoFactory
import choreo.trajectory.TrajectorySample
import java.util.function.Consumer
import java.util.function.Supplier
import org.wpilib.command2.Command as V2Command
import org.wpilib.command2.Subsystem
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.math.geometry.Pose2d

/**
 * Runs Choreo's standalone commands on the Commands v3 scheduler.
 *
 * The v2 subsystem is only a requirement token. The returned commands reserve
 * [drive] in v3 and invoke the v2 lifecycle directly, without running a second
 * scheduler. Compose these commands using v3 coroutines or command groups.
 * Choreo AutoRoutine triggers, event bindings, and spawnCmd are not supported.
 */
class ChoreoAutoFactory<Sample : TrajectorySample<Sample>>(
    poseSupplier: Supplier<Pose2d>,
    resetOdometry: Consumer<Pose2d>,
    controller: Consumer<Sample>,
    useAllianceFlipping: Boolean,
    private val drive: Mechanism,
) {
    private val subsystem = object : Subsystem {}
    private val factory =
        AutoFactory(
            poseSupplier,
            resetOdometry,
            controller,
            useAllianceFlipping,
            subsystem,
        )

    fun trajectoryCmd(name: String): Command =
        adapt(factory.trajectoryCmd(name))

    fun trajectoryCmd(name: String, splitIndex: Int): Command =
        adapt(factory.trajectoryCmd(name, splitIndex) { it })

    fun resetOdometry(name: String): Command =
        adapt(factory.resetOdometry(name))

    fun resetOdometry(name: String, splitIndex: Int): Command =
        adapt(factory.resetOdometry(name, splitIndex))

    fun warmupCmd(): Command = adapt(factory.warmupCmd())

    private fun adapt(command: V2Command): Command {
        require(command.requirements.all { it === subsystem }) {
            "Choreo command has an unmapped v2 subsystem requirement"
        }
        val lifecycle = V2CommandLifecycle(command)
        return Command.requiring(drive)
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
