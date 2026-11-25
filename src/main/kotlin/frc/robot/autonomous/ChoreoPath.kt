package frc.robot.autonomous

import choreo.Choreo
import choreo.trajectory.EventMarker
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.commands.invoke
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.system.Timer

/**
 * A Choreo trajectory from `deploy/choreo`.
 *
 * Don't extend this directly: KSP generates an abstract `<Name>Path` class for
 * every `.traj` file, with an abstract function per event marker. Implement it
 * as an object to bind the events:
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
     * Follows the path, flipped for the red alliance, running each event
     * marker's binding in this command's coroutine when its time is reached.
     *
     * A binding that awaits a command blocks the path: the drive stops and the
     * path's timer pauses until the binding returns. Fork a command to run it
     * alongside the path; forked commands are canceled when the path ends.
     *
     * @param mirrored Mirror the path across the field's long axis, to run a
     *   path drawn for one side of the field on the other side.
     */
    fun follow(mirrored: Boolean = false): Command =
        drive {
                val trajectory = resolve(mirrored)
                Logger.recordOutput("Autonomous/Trajectory", *trajectory.poses)

                val pendingEvents =
                    ArrayDeque(trajectory.events().sortedBy { it.timestamp })
                val timer = Timer().apply { start() }
                while (timer.get() < trajectory.totalTime) {
                    val reached = pendingEvents.popReached(timer.get())
                    if (reached.isNotEmpty()) {
                        timer.stop()
                        drive.stop()
                        reached.forEach { runEvent(it.event) }
                        timer.start()
                    }
                    drive.followTrajectory(
                        trajectory.sampleAt(timer.get(), false).get()
                    )
                    yield()
                }
                drive.stop()
                pendingEvents.forEach { runEvent(it.event) }
            }
            .whenCanceled(drive::stop)
            .named(if (mirrored) "$name (Mirrored)" else name)

    /**
     * Resets odometry to the start of the path, as [follow] with the same
     * [mirrored] would drive it.
     */
    fun resetOdometry(mirrored: Boolean = false): Command =
        drive {
                drive.resetOdometry(
                    resolve(mirrored).getInitialPose(false).get()
                )
            }
            .named("$name/resetOdometry")

    private fun resolve(mirrored: Boolean): Trajectory<SwerveSample> =
        trajectory
            .let { if (mirrored) it.mirrorY() else it }
            .let { if (IS_RED) it.flipped() else it }

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
