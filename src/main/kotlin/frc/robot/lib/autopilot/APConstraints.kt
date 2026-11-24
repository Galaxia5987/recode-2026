// Copied from [Autopilot](https://github.com/therekrab/autopilot) by Nolan Barker
// Licensed under the MIT License

package frc.robot.lib.autopilot

import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Holds constraint information for an Autopilot action.
 *
 * Constraints are max velocity, acceleration, and jerk. The defaults are unlimited.
 *
 * @property velocity The maximum velocity that Autopilot will demand, in m/s
 * @property acceleration The maximum acceleration that Autopilot will use to correct initial
 *   velocities, in m/s^2
 * @property jerk The maximum jerk that Autopilot will use to decelerate at the end of an action, in
 *   m/s^3
 */
data class APConstraints(
    val velocity: Double = Double.POSITIVE_INFINITY,
    val acceleration: Double = Double.POSITIVE_INFINITY,
    val jerk: Double = Double.POSITIVE_INFINITY,
) {
    // These values represent the "cutoff" parameters that Autopilot uses to determine what should
    // be happening during the end behavior of a path. These are always the same for a constant
    // constraint, so to save the most computational time, they're precomputed with the constraint,
    // rather than each time Autopilot.calculate is called.

    /** Cutoff distance */
    private val x0 = acceleration.pow(3.0) / (18.0 * jerk * jerk)

    /** Velocity at cutoff distance */
    private val v0 = jerkConstrainedVelocity(x0)

    /** Returns a copy of this object with the given max velocity, in m/s. */
    fun withVelocity(newVelocity: Double) = copy(velocity = newVelocity)

    /**
     * Returns a copy of this object with the given acceleration, in m/s^2. Autopilot's acceleration
     * is used at the beginning and end of an action.
     */
    fun withAcceleration(newAcceleration: Double) =
        copy(acceleration = newAcceleration)

    /**
     * Returns a copy of this object with the given max jerk, in m/s^3. Higher values mean a faster
     * deceleration. Autopilot's jerk is used at the end of an action (not relevant to Autopilot's
     * start behavior).
     */
    fun withJerk(newJerk: Double) = copy(jerk = newJerk)

    /**
     * Determines the maximum velocity required to travel the given distance and end at 0 m/s.
     *
     * @param dist The distance to travel, in meters
     */
    internal fun calculateMaxVelocity(dist: Double) =
        if (dist > x0) accelerationConstrainedVelocity(dist)
        else jerkConstrainedVelocity(dist)

    private fun accelerationConstrainedVelocity(dist: Double) =
        sqrt(v0 * v0 + 2.0 * acceleration * (dist - x0))

    private fun jerkConstrainedVelocity(dist: Double) =
        (4.5 * dist.pow(2.0) * jerk).pow(1.0 / 3.0)
}
