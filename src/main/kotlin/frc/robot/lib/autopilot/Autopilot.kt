// Copied from [Autopilot](https://github.com/therekrab/autopilot) by Nolan
// Barker
// Licensed under the MIT License

package frc.robot.lib.autopilot

import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.getAngleOrDefault
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.rad
import frc.robot.lib.toTranslation2d
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.ln
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.units.measure.LinearVelocity

/**
 * Autopilot tries to drive a robot to a goal in 2-D space.
 *
 * Autopilot is a stateless algorithm; as such, it does not "think ahead" and
 * cannot avoid obstacles. Any math that Autopilot needs is already worked out
 * such that only a small amount of computation is necessary on the fly.
 * Autopilot is designed to be used in a drivetrain's control loop, where the
 * current state of the robot is passed in, and the next velocity is returned.
 *
 * @param profile The profile that the autopilot will use for all actions.
 */
class Autopilot(private val profile: APProfile) {
    /**
     * Returns the next field relative velocity for the trajectory.
     *
     * @param current The robot's current position.
     * @param robotRelativeSpeeds The robot's current **robot relative**
     *   [ChassisVelocities].
     * @param target The target the robot should drive towards.
     * @return an [APResult] containing the next velocity and target angle
     */
    fun calculate(
        current: Pose2d,
        robotRelativeSpeeds: ChassisVelocities,
        target: APTarget,
    ): APResult {
        val offset =
            toTargetCoordinateFrame(
                target.reference.translation - current.translation,
                target,
            )

        if (offset == Translation2d.ZERO) {
            return APResult(0.mps, 0.mps, target.reference.rotation)
        }

        val fieldRelativeSpeeds =
            robotRelativeSpeeds.toTranslation2d().rotateBy(current.rotation)

        val initial = toTargetCoordinateFrame(fieldRelativeSpeeds, target)
        val disp = offset.norm
        val goal =
            if (target.entryAngle == null || disp < profile.beelineRadius[m]) {
                val towardsTarget = offset / disp
                towardsTarget *
                    (profile.constraints.calculateMaxVelocity(disp) +
                        target.velocity)
            } else {
                calculateSwirlyVelocity(offset, target)
            }
        val velocity = toGlobalCoordinateFrame(correct(initial, goal), target)
        val rotation = getRotationTarget(current.rotation, target, disp)
        return APResult(velocity.x.mps, velocity.y.mps, rotation)
    }

    /**
     * Turns any other coordinate frame into a coordinate frame with positive x
     * meaning in the direction of the target's entry angle, if applicable
     * (otherwise no change to angles).
     */
    private fun toTargetCoordinateFrame(
        coords: Translation2d,
        target: APTarget,
    ) = coords.rotateBy(-(target.entryAngle ?: Rotation2d.ZERO))

    /**
     * Turns a translation from a target-relative coordinate frame to a global
     * coordinate frame.
     */
    private fun toGlobalCoordinateFrame(
        coords: Translation2d,
        target: APTarget,
    ) = coords.rotateBy(target.entryAngle ?: Rotation2d.ZERO)

    /**
     * Attempts to drive the initial translation to the goal translation using
     * the parameters for acceleration given in the profile.
     *
     * @param initial The initial translation to drive from
     * @param goal The goal translation to drive to
     */
    private fun correct(
        initial: Translation2d,
        goal: Translation2d,
    ): Translation2d {
        val angleOffset = goal.getAngleOrDefault()
        val adjustedGoal = goal.rotateBy(-angleOffset)
        val adjustedInitial = initial.rotateBy(-angleOffset)
        val initialI = adjustedInitial.x
        // we cap the adjusted I because we'd rather adjust now than overshoot.
        val goalI = adjustedGoal.x.coerceAtMost(profile.constraints.velocity)
        val adjustedI =
            minOf(
                goalI,
                push(initialI, goalI, profile.constraints.acceleration),
            )
        return Translation2d(adjustedI, 0.0).rotateBy(angleOffset)
    }

    /**
     * Using the provided acceleration, "pushes" the start point towards the end
     * point.
     *
     * This is used for ensuring that changes in velocity are within the
     * acceleration threshold.
     */
    private fun push(start: Double, end: Double, accel: Double): Double {
        val maxChange = accel * DT
        return when {
            abs(start - end) < maxChange -> end
            start > end -> start - maxChange
            else -> start + maxChange
        }
    }

    /**
     * Uses the swirly method to calculate the correct velocities for the robot,
     * respecting entry angles.
     *
     * @param offset The offset from the robot to the target, in the target's
     *   coordinate frame
     * @param target The target that Autopilot is trying to reach
     */
    private fun calculateSwirlyVelocity(
        offset: Translation2d,
        target: APTarget,
    ): Translation2d {
        val disp = offset.norm
        val theta = offset.getAngleOrDefault()
        val rads = theta.radians
        val dist = calculateSwirlyLength(rads, disp)
        val vx = theta.cos - rads * theta.sin
        val vy = rads * theta.cos + theta.sin
        return Translation2d(vx, vy) / hypot(vx, vy) * // normalize
            (profile.constraints.calculateMaxVelocity(dist) + target.velocity)
        // and scale to new length
    }

    /**
     * Using a precomputed integral, returns the length of the path that the
     * swirly method generates.
     *
     * More specifically, this calculates the arc length of the polar curve
     * r=theta from the given angle to zero, then scales it to match the current
     * state.
     *
     * @param theta The angle of the offset from the robot to the target, in
     *   radians
     * @param radius The normalized offset from the robot to the target, in
     *   meters
     */
    private fun calculateSwirlyLength(theta: Double, radius: Double): Double {
        if (theta == 0.0) return radius
        val absTheta = abs(theta)
        val hypot = hypot(absTheta, 1.0)
        val u1 = radius * hypot
        val u2 = radius * ln(absTheta + hypot) / absTheta
        return 0.5 * (u1 + u2)
    }

    /**
     * Returns the target's rotation if the robot is within a specified rotation
     * radius; otherwise, returns the current rotation of the robot.
     *
     * @param current The current rotation of the robot.
     * @param target The [APTarget] that Autopilot is trying to reach.
     * @param dist The distance from the robot to the target.
     */
    private fun getRotationTarget(
        current: Rotation2d,
        target: APTarget,
        dist: Double,
    ): Rotation2d {
        val radius = target.rotationRadius?.get(m)
        return if (radius == null || radius > dist) target.reference.rotation
        else current
    }

    /**
     * Returns whether the given pose is within the tolerance of the [APTarget].
     *
     * @param current The current pose of the robot.
     * @param target The [APTarget] to check against.
     * @return true if Autopilot has reached the target.
     */
    fun atTarget(current: Pose2d, target: APTarget): Boolean {
        val goal = target.reference
        val okXY =
            hypot(current.x - goal.x, current.y - goal.y) <= profile.errorXY[m]
        val okTheta =
            abs((current.rotation - goal.rotation).radians) <=
                profile.errorTheta[rad]
        return okXY && okTheta
    }

    /** The computed motion from a call to [Autopilot.calculate]. */
    data class APResult(
        val vx: LinearVelocity,
        val vy: LinearVelocity,
        val targetAngle: Rotation2d,
    )

    private companion object {
        const val DT = 0.020
    }
}
