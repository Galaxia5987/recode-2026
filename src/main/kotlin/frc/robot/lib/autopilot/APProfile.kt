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
data class APProfile(
    val constraints: APConstraints,
    val errorXY: Distance = Meters.of(0.0),
    val errorTheta: Angle = Rotations.of(0.0),
    val beelineRadius: Distance = Meters.of(0.0),
)
