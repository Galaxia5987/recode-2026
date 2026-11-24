// Copied from [Autopilot](https://github.com/therekrab/autopilot) by Nolan Barker
// Licensed under the MIT License

package frc.robot.lib.autopilot

import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.units.measure.Distance

/**
 * The goal end state of an Autopilot action.
 *
 * A target needs a reference [Pose2d], but can optionally have a specified entry angle (the angle
 * which the robot will start to approach the target with, at the start of the path) and rotation
 * radius. A target may also specify an end velocity.
 *
 * @property reference The reference pose for this target.
 * @property entryAngle The desired entry angle, if any.
 * @property velocity The desired end velocity when the robot approaches the target. Note that if
 *   the robot does not reach this velocity, no issues will be thrown.
 * @property rotationRadius The distance from the target pose that rotation goals are respected. By
 *   default, rotation goals are always respected.
 */
data class APTarget(
    val reference: Pose2d,
    val entryAngle: Rotation2d? = null,
    val velocity: Double = 0.0,
    val rotationRadius: Distance? = null,
) {
    /** Returns a copy of this target with the given reference pose. */
    fun withReference(reference: Pose2d) = copy(reference = reference)

    /** Returns a copy of this target with the given entry angle. */
    fun withEntryAngle(entryAngle: Rotation2d) = copy(entryAngle = entryAngle)

    /** Returns a copy of this target with the given end velocity. */
    fun withVelocity(velocity: Double) = copy(velocity = velocity)

    /** Returns a copy of this target with the given rotation radius. */
    fun withRotationRadius(radius: Distance) = copy(rotationRadius = radius)

    /**
     * Returns a copy of this target, without the entry angle set.
     *
     * This is useful if trying to make two different targets with and without entry angle set.
     */
    fun withoutEntryAngle() = copy(entryAngle = null)
}
