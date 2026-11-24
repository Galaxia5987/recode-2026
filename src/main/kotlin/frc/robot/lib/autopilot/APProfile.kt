// Copied from [Autopilot](https://github.com/therekrab/autopilot) by Nolan Barker
// Licensed under the MIT License

package frc.robot.lib.autopilot

import org.wpilib.units.Units.Meters
import org.wpilib.units.Units.Rotations
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

/**
 * A profile that determines how Autopilot approaches a target.
 *
 * The constraints property of the profile limits the robot's behavior.
 *
 * Acceptable error for the controller (both translational and rotational) are stored here.
 *
 * The "beeline radius" determines the distance at which the robot drives directly at the target and
 * no longer respects entry angle. This is helpful because if the robot overshoots by a small
 * amount, that error should not cause the robot do completely circle back around.
 *
 * Tolerated error and beeline radius default to zero.
 */
class APProfile(
    var constraints: APConstraints,
    var errorXY: Distance = Meters.of(0.0),
    var errorTheta: Angle = Rotations.of(0.0),
    var beelineRadius: Distance = Meters.of(0.0),
) {
    /** Modifies this profile's tolerated error in the XY plane and returns itself. */
    fun withErrorXY(errorXY: Distance) = apply { this.errorXY = errorXY }

    /** Modifies this profile's tolerated angular error and returns itself. */
    fun withErrorTheta(errorTheta: Angle) = apply { this.errorTheta = errorTheta }

    /** Modifies this profile's path generation constraints and returns itself. */
    fun withConstraints(constraints: APConstraints) = apply {
        this.constraints = constraints
    }

    /**
     * Modifies this profile's beeline radius and returns itself.
     *
     * The beeline radius is a distance where, under that range, entry angle is no longer respected.
     * This prevents small overshoots from causing the robot to make a full arc and instead correct
     * itself.
     */
    fun withBeelineRadius(beelineRadius: Distance) = apply {
        this.beelineRadius = beelineRadius
    }
}
