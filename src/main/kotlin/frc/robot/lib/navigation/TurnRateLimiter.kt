package frc.robot.lib.navigation

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d

/**
 * Smooths a stream of velocity commands so the robot can actually follow them.
 *
 * It limits speed and heading separately, so the robot keeps its speed through
 * a bend instead of cutting across it (a plain 2D vector limit shortens the
 * velocity in every turn):
 * - Speeding up is limited to [acceleration].
 * - Slowing down happens immediately, like Autopilot's `correct()`. The speed
 *   asked for already includes braking (the Autopilot profile and the bend
 *   limit), so limiting it again only causes overshoot.
 * - The heading may turn at most `lateralAcceleration / speed` rad/s, the
 *   sharpest turn the tyres can hold at that speed.
 * - If the goal points away from the current heading, the speed is scaled by
 *   the cosine of the difference (zero past 90°), so the robot slows down and
 *   turns instead of circling the goal.
 *
 * Below [minSpeedForHeading] the heading is meaningless, so the whole velocity
 * vector is moved toward the goal by at most `acceleration · dt` instead.
 *
 * The limiter starts from the last command it returned, not from the measured
 * velocity, so encoder noise does not feed back into the command.
 *
 * @param acceleration Maximum speed-up, in m/s².
 * @param lateralAcceleration Maximum sideways acceleration in turns, in m/s².
 * @param dt The loop period, in seconds.
 * @param minSpeedForHeading Speed below which the plain vector limit is used,
 *   in m/s.
 */
class TurnRateLimiter(
    private val acceleration: Double,
    private val lateralAcceleration: Double,
    private val dt: Double = 0.02,
    private val minSpeedForHeading: Double = 0.3,
) {
    /** The last returned velocity, field relative, in m/s. */
    var lastVelocity: Translation2d = Translation2d.ZERO
        private set

    /**
     * Restarts the limiter from a known velocity, e.g. the robot's measured
     * velocity when a command starts.
     *
     * @param velocity The field-relative velocity to start from, in m/s.
     */
    fun reset(velocity: Translation2d) {
        lastVelocity = velocity
    }

    /**
     * Moves the last velocity toward the goal within the limits.
     *
     * @param goal The wanted field-relative velocity, in m/s.
     * @return The velocity to command this loop, field relative, in m/s.
     */
    fun calculate(goal: Translation2d): Translation2d {
        val speed = lastVelocity.norm
        val goalSpeed = goal.norm
        lastVelocity =
            if (speed < minSpeedForHeading || goalSpeed < 1e-6) {
                limitVector(goal)
            } else {
                limitSpeedAndHeading(speed, goal, goalSpeed)
            }
        return lastVelocity
    }

    /**
     * Turns toward the goal's heading and changes speed, each within limits.
     */
    private fun limitSpeedAndHeading(
        speed: Double,
        goal: Translation2d,
        goalSpeed: Double,
    ): Translation2d {
        val heading = atan2(lastVelocity.y, lastVelocity.x)
        val headingError = wrapAngle(atan2(goal.y, goal.x) - heading)
        val maxTurn = lateralAcceleration / speed * dt
        val newHeading = heading + headingError.coerceIn(-maxTurn, maxTurn)

        // Pointing the wrong way: slow down instead of orbiting the goal.
        val wantedSpeed = goalSpeed * max(cos(headingError), 0.0)
        // Speed up within the limit, brake immediately.
        val newSpeed = minOf(speed + acceleration * dt, wantedSpeed)

        return Translation2d(newSpeed, Rotation2d(newHeading))
    }

    /**
     * Moves the whole velocity vector toward the goal by at most `acceleration
     * · dt`, but never faster than the goal speed.
     */
    private fun limitVector(goal: Translation2d): Translation2d {
        val delta = goal - lastVelocity
        val maxChange = acceleration * dt
        val limited =
            if (delta.norm <= maxChange) goal
            else lastVelocity + delta * (maxChange / delta.norm)
        return if (limited.norm > goal.norm) {
            limited * (goal.norm / limited.norm)
        } else {
            limited
        }
    }

    /** Wraps an angle in radians to (-π, π]. */
    private fun wrapAngle(radians: Double): Double =
        atan2(sin(radians), cos(radians))
}
