package frc.robot.autonomous

import choreo.Choreo
import choreo.trajectory.EventMarker
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import frc.robot.drive
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Coroutine
import org.wpilib.system.Timer

/**
 * A Choreo trajectory from `deploy/choreo`.
 *
 * Don't extend this directly: KSP generates an abstract `<Name>Path` class for
 * every `.traj` file, with an abstract function per event marker. Implement it
 * as an object to bind the events and add it to the auto chooser:
 * ```kotlin
 * object StartToFuel : StartToFuelPath() {
 *     override fun Coroutine.intakeDown() = Intake.open().fork()
 *     override fun Coroutine.shoot() = +Shooter.shoot()
 * }
 * ```
 */
abstract class ChoreoPath(val name: String) {
    private val trajectory: Trajectory<SwerveSample> by lazy {
        Choreo.loadTrajectory<SwerveSample>(name).orElseThrow {
            IllegalStateException("Failed to load Choreo trajectory '$name'")
        }
    }

    protected abstract fun Coroutine.runEvent(event: String)

    /**
     * Follows the path as part of [routine], running each event marker's
     * binding in the routine's coroutine when its time is reached.
     *
     * A binding that awaits a command blocks the path: the drive stops and the
     * path's timer pauses until the binding returns. Fork a command to run it
     * alongside the path; forked commands live until the routine ends.
     */
    context(routine: AutoRoutine, coroutine: Coroutine)
    fun follow() {
        val trajectory = routine.resolve(trajectory)
        routine.start(trajectory.getInitialPose(false).get())
        Logger.recordOutput("Autonomous/Trajectory", *trajectory.poses)

        val pendingEvents =
            ArrayDeque(trajectory.events().sortedBy { it.timestamp })
        val timer = Timer().apply { start() }
        while (timer.get() < trajectory.totalTime) {
            val reached = pendingEvents.popReached(timer.get())
            if (reached.isNotEmpty()) {
                timer.stop()
                drive.stop()
                reached.forEach { coroutine.runEvent(it.event) }
                timer.start()
            }
            drive.followTrajectory(
                trajectory.sampleAt(timer.get(), false).get()
            )
            coroutine.yield()
        }
        drive.stop()
        pendingEvents.forEach { coroutine.runEvent(it.event) }
    }

    private fun ArrayDeque<EventMarker>.popReached(
        time: Double
    ): List<EventMarker> {
        val reached = mutableListOf<EventMarker>()
        while (firstOrNull()?.let { it.timestamp <= time } == true) {
            reached += removeFirst()
        }
        return reached
    }
}
