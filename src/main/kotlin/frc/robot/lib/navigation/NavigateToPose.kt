package frc.robot.lib.navigation

import frc.robot.drive
import frc.robot.field.FIELD_OBSTACLES
import frc.robot.lib.FlippingUtil
import frc.robot.lib.align.runToPose
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.subsystems.drive.TunerConstants
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance

/**
 * Half the robot's footprint.
 *
 * TODO: Measure. Half the bumper width lets corners clip when the robot is
 *   rotated; half the bumper diagonal is always safe but closes tighter gaps.
 */
private val ROBOT_RADIUS: Distance = 0.45.m

/** Extra room on top of [ROBOT_RADIUS] when checking a straight line. */
private val SAFETY_MARGIN: Distance = 0.05.m

/**
 * How far outside each obstacle the corner waypoints sit. Larger than the
 * line-of-sight clearance, so cutting a corner never reaches the obstacle.
 */
private val CORNER_OFFSET: Distance = 0.7.m

/** Move on to the next leg once this close to a corner. */
private val SWITCH_DISTANCE: Distance = 0.3.m

/** Sideways acceleration the tyres can hold in a corner, in m/s². */
private const val LATERAL_ACCELERATION = 5.0

private val planner =
    CornerPlanner(
        obstacles = FIELD_OBSTACLES,
        fieldLength = FlippingUtil.fieldSizeX.m,
        fieldWidth = FlippingUtil.fieldSizeY.m,
        clearance = ROBOT_RADIUS + SAFETY_MARGIN,
        cornerOffset = CORNER_OFFSET,
    )

/**
 * Drives to a pose, going around the field's obstacles.
 *
 * 1. Plans the shortest route through obstacle corners with [CornerPlanner].
 * 2. Drives to each corner with [runToPose], passing through it at the speed
 *    the turn allows ([CornerPlanner.cornerSpeed]) and moving on to the next
 *    leg [SWITCH_DISTANCE] before reaching it. If a corner stops being visible
 *    on the way (e.g. the robot was pushed), the route is planned again from
 *    where the robot is.
 * 3. Drives the last leg with a normal [runToPose], stopping exactly on the
 *    goal.
 *
 * When the straight line to the goal is clear, or no route exists, this is just
 * [runToPose].
 *
 * ### Example
 *
 * ```kotlin
 * driverController.cross().whileTrue(navigateToPose(shootingPose))
 * ```
 *
 * @param goal The pose to drive to, in blue-origin field coordinates (flip it
 *   for the red alliance before passing it in).
 * @param entryAngle The direction to approach the goal from, passed to the
 *   final [runToPose].
 * @param rotationRadius Passed to the final [runToPose].
 * @return The command, requiring the drive.
 */
fun navigateToPose(
    goal: Pose2d,
    entryAngle: Rotation2d? = null,
    rotationRadius: Distance? = null,
): Command =
    drive {
            var route = planRoute(goal)
            while (route.size > 1) {
                val corner = route[0]
                val speed =
                    CornerPlanner.cornerSpeed(
                        before = drive.pose.translation,
                        corner = corner,
                        after = route[1],
                        lateralAcceleration = LATERAL_ACCELERATION,
                        turnRadius = CORNER_OFFSET[m],
                        maxSpeed = TunerConstants.kSpeedAt12Volts[mps],
                    )
                var cornerHidden = false
                +runToPose(
                        Pose2d(corner, goal.rotation),
                        endVelocity = speed.mps,
                        passThrough = { pose ->
                            cornerHidden =
                                !planner.isClear(pose.translation, corner)
                            cornerHidden ||
                                pose.translation.getDistance(corner) <
                                    SWITCH_DISTANCE[m]
                        },
                    )
                    .named("Navigation/Corner")
                route = if (cornerHidden) planRoute(goal) else route.drop(1)
            }
            +runToPose(goal, entryAngle, rotationRadius = rotationRadius)
                .named("Navigation/FinalApproach")
        }
        .named("Navigation/NavigateToPose")

/**
 * Plans a route from the robot's current position and logs it.
 *
 * @param goal The goal pose.
 * @return The waypoints, ending with the goal. Just the goal if the line is
 *   clear or no route exists.
 */
private fun planRoute(goal: Pose2d): List<Translation2d> {
    val route =
        planner.plan(drive.pose.translation, goal.translation)
            ?: listOf(goal.translation)
    Logger.recordOutput(
        "Navigation/Route",
        *route.map { Pose2d(it, goal.rotation) }.toTypedArray(),
    )
    return route
}
