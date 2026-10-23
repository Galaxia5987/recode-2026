package frc.robot.lib.commands

import choreo.auto.AutoTrajectory
import choreo.trajectory.Trajectory
import choreo.trajectory.TrajectorySample
import java.util.Optional
import java.util.function.BooleanSupplier
import org.wpilib.command3.Command
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Translation2d

/** Choreo trajectory operations with v3 commands and routine-local triggers. */
class ChoreoAutoTrajectory<Sample : TrajectorySample<Sample>>
internal constructor(
    private val factory: ChoreoAutoFactory<Sample>,
    internal val delegate: AutoTrajectory,
    private val routine: ChoreoAutoRoutine<Sample>?,
) {
    fun cmd(): Command =
        factory.adapt(delegate.cmd()).also { routine?.track(it) }

    fun resetOdometry(): Command = factory.adapt(delegate.resetOdometry())

    /**
     * Starts the trajectory as a child of its routine, rather than of this
     * one-shot command.
     */
    fun spawnCmd(): Command {
        val owner =
            requireNotNull(routine) { "spawnCmd requires a routine trajectory" }
        return Command.noRequirements { owner.spawn(cmd()) }
            .named("Spawn/${getRawTrajectory().name()}")
    }

    fun getRawTrajectory(): Trajectory<Sample> =
        delegate.getRawTrajectory<Sample>()

    fun getInitialPose(): Optional<Pose2d> = delegate.initialPose

    fun getFinalPose(): Optional<Pose2d> = delegate.finalPose

    fun mirrorX(): ChoreoAutoTrajectory<Sample> =
        wrap(delegate.mirrorX<Sample>())

    fun mirrorY(): ChoreoAutoTrajectory<Sample> =
        wrap(delegate.mirrorY<Sample>())

    fun rotateAround(): ChoreoAutoTrajectory<Sample> =
        wrap(delegate.rotateAround<Sample>())

    private fun wrap(raw: AutoTrajectory): ChoreoAutoTrajectory<Sample> =
        routine?.wrap(raw) ?: ChoreoAutoTrajectory(factory, raw, null)

    private fun trigger(condition: BooleanSupplier): ChoreoTrigger =
        requireNotNull(routine) {
                "Trajectory triggers require a routine trajectory"
            }
            .trigger(condition)

    fun active(): ChoreoTrigger = trigger(delegate.active())

    fun inactive(): ChoreoTrigger = trigger(delegate.inactive())

    fun done(): ChoreoTrigger = trigger(delegate.done())

    fun doneDelayed(seconds: Double): ChoreoTrigger =
        trigger(delegate.doneDelayed(seconds))

    fun doneFor(seconds: Double): ChoreoTrigger =
        trigger(delegate.doneFor(seconds))

    fun recentlyDone(): ChoreoTrigger = trigger(delegate.recentlyDone())

    fun chain(other: ChoreoAutoTrajectory<Sample>) {
        done().onTrue(other.cmd())
    }

    fun atTime(seconds: Double): ChoreoTrigger =
        trigger(delegate.atTime(seconds))

    fun atTime(eventName: String): ChoreoTrigger =
        trigger(delegate.atTime(eventName))

    fun atTimeBeforeEnd(seconds: Double): ChoreoTrigger =
        trigger(delegate.atTimeBeforeEnd(seconds))

    fun atPose(
        pose: Pose2d,
        toleranceMeters: Double,
        toleranceRadians: Double,
    ): ChoreoTrigger =
        trigger(delegate.atPose(pose, toleranceMeters, toleranceRadians))

    fun atPose(
        eventName: String,
        toleranceMeters: Double,
        toleranceRadians: Double,
    ): ChoreoTrigger =
        trigger(delegate.atPose(eventName, toleranceMeters, toleranceRadians))

    fun atTranslation(
        translation: Translation2d,
        toleranceMeters: Double,
    ): ChoreoTrigger =
        trigger(delegate.atTranslation(translation, toleranceMeters))

    fun atTranslation(
        eventName: String,
        toleranceMeters: Double,
    ): ChoreoTrigger =
        trigger(delegate.atTranslation(eventName, toleranceMeters))

    fun collectEventTimes(eventName: String): DoubleArray =
        delegate.collectEventTimes(eventName)

    fun collectEventPoses(eventName: String): Array<Pose2d> =
        delegate.collectEventPoses(eventName)
}
