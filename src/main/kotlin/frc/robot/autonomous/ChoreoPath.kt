package frc.robot.autonomous

import choreo.Choreo
import choreo.trajectory.EventMarker
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.commands.fork
import frc.robot.lib.commands.invoke
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.system.Timer

/**
 * A Choreo trajectory from `deploy/choreo`.
 *
 * Don't extend this directly: KSP generates an abstract `<Name>Path` class for
 * every `.traj` file, with an abstract command function per event marker.
 * Implement it as an object to bind the events and add it to the auto chooser:
 * ```kotlin
 * object StartToFuel : StartToFuelPath() {
 *     override fun intake() = Intake.open()
 * }
 * ```
 */
abstract class ChoreoPath(val name: String) {
    protected abstract val eventCommands: Map<String, () -> Command>

    private val trajectory: Trajectory<SwerveSample> by lazy {
        Choreo.loadTrajectory<SwerveSample>(name).orElseThrow {
            IllegalStateException("Failed to load Choreo trajectory '$name'")
        }
    }

    /**
     * Resets odometry to the start of the path, follows it, and forks each
     * event marker's command when its time is reached. The path is flipped for
     * the red alliance.
     *
     * Event commands are children of this command, so they are canceled when
     * the path ends and must not require the drive.
     *
     * @param mirrored Mirror the path across the field's long axis, to run a
     *   path drawn for one side of the field on the other side.
     */
    fun follow(mirrored: Boolean = false): Command =
        drive {
                val trajectory = resolveTrajectory(mirrored)
                val pendingEvents =
                    ArrayDeque(trajectory.events().sortedBy { it.timestamp })
                Logger.recordOutput(
                    "Autonomous/Trajectory",
                    *trajectory.poses,
                )
                drive.resetOdometry(trajectory.getInitialPose(false).get())

                val timer = Timer().apply { start() }
                while (timer.get() < trajectory.totalTime) {
                    pendingEvents.popReached(timer.get()).forEach {
                        eventCommands.getValue(it.event)().fork()
                    }
                    drive.followTrajectory(
                        trajectory.sampleAt(timer.get(), false).get()
                    )
                    yield()
                }
                drive.stop()
            }
            .named(if (mirrored) "$name (Mirrored)" else name)

    private fun resolveTrajectory(mirrored: Boolean) =
        trajectory
            .let { if (mirrored) it.mirrorY() else it }
            .let { if (IS_RED) it.flipped() else it }

    private fun ArrayDeque<EventMarker>.popReached(
        time: Double
    ): List<EventMarker> = buildList {
        while (firstOrNull()?.let { it.timestamp <= time } == true) {
            add(removeFirst())
        }
    }
}
