package frc.robot.autonomous

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.commands.invoke
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.math.geometry.Pose2d

/** The state shared by the paths followed in one [autoRoutine]. */
class AutoRoutine internal constructor(private val mirrored: Boolean) {
    private var started = false

    /**
     * Mirrors [trajectory] across the field's long axis if the routine is
     * mirrored, and flips it for the red alliance.
     */
    internal fun resolve(
        trajectory: Trajectory<SwerveSample>
    ): Trajectory<SwerveSample> =
        trajectory
            .let { if (mirrored) it.mirrorY() else it }
            .let { if (IS_RED) it.flipped() else it }

    /** Resets odometry to [pose] when the routine's first path starts. */
    internal fun start(pose: Pose2d) {
        if (started) return
        drive.resetOdometry(pose)
        started = true
    }
}

/**
 * Creates an autonomous routine that requires the drive and runs [block], where
 * paths are followed one after another with [ChoreoPath.follow]:
 * ```kotlin
 * fun twoPiece(mirrored: Boolean) =
 *     autoRoutine("Two Piece", mirrored) {
 *         StartToFuel.follow()
 *         +Shooter.shoot()
 *         FuelToHub.follow()
 *     }
 * ```
 *
 * Odometry is reset to the start of the first path.
 *
 * @param mirrored Mirror every path across the field's long axis, to run a
 *   routine drawn for one side of the field on the other side.
 */
fun autoRoutine(
    name: String,
    mirrored: Boolean = false,
    block: context(AutoRoutine) Coroutine.() -> Unit,
): Command = drive { block(AutoRoutine(mirrored), this) }.named(name)
