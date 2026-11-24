package frc.robot.lib.navigation

import frc.robot.drive
import frc.robot.field.FIELD_OBSTACLES
import frc.robot.lib.FlippingUtil
import frc.robot.lib.TuneablePIDController
import frc.robot.lib.align.runToPose
import frc.robot.lib.autopilot.APConstraints
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.toTranslation2d
import frc.robot.subsystems.drive.TunerConstants
import kotlin.math.PI
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.units.measure.Distance

/**
 * Half the robot's footprint, used to keep its center away from obstacles.
 *
 * TODO: Measure. Half the bumper width lets corners clip when the robot is
 *   rotated; half the bumper diagonal is always safe but closes tighter gaps.
 */
private val ROBOT_RADIUS: Distance = 0.45.m

/** Extra clearance on top of [ROBOT_RADIUS] for tracking error. */
private val SAFETY_MARGIN: Distance = 0.05.m

private val NAV_GRID_CONFIG =
    NavGridConfig(
        fieldLength = FlippingUtil.fieldSizeX.m,
        fieldWidth = FlippingUtil.fieldSizeY.m,
        inflation = ROBOT_RADIUS + SAFETY_MARGIN,
    )

/**
 * Speed limits while following the field. Acceleration and jerk match
 * [runToPose]'s Autopilot so the speed does not jump at the handover.
 */
private val NAV_CONSTRAINTS =
    APConstraints(
        velocity = TunerConstants.kSpeedAt12Volts[mps],
        acceleration = 5.5,
        jerk = 300.0,
    )

/** Sideways acceleration the tyres can hold in a bend, in m/s². */
private const val LATERAL_ACCELERATION = 5.0

/**
 * When the cost-to-go drops below this, the route ahead is a clear straight
 * line and [runToPose] takes over to stop exactly on the goal.
 */
private val HANDOVER_DISTANCE: Distance = 0.6.m

/**
 * Navigation fields for every goal, shared by all [navigateToPose] commands.
 *
 * Call [NavFieldCache.precompute] with fixed goals at robot init so they are
 * ready when first used.
 */
val navFieldCache =
    NavFieldCache({
        NavGrid(NAV_GRID_CONFIG, FIELD_OBSTACLES.map { it.toNavObstacle() })
    })

private val headingController =
    TuneablePIDController("navigationHeadingPIDController", 8.0, 0.0, 0.5)

private val limiter =
    TurnRateLimiter(NAV_CONSTRAINTS.acceleration, LATERAL_ACCELERATION)

/**
 * Drives to a pose, going around the field's obstacles.
 *
 * 1. Gets the [NavField] for the goal from [navFieldCache], waiting (with the
 *    drive stopped) if it is still being computed.
 * 2. Follows the field downhill: direction from [NavField.downhill], speed from
 *    the Autopilot braking profile on the remaining cost-to-go and from
 *    [NavField.cornerSpeedLimit], smoothed by a [TurnRateLimiter]. The heading
 *    turns toward the goal's rotation the whole way.
 * 3. Once the cost-to-go is below [HANDOVER_DISTANCE], runs [runToPose] to stop
 *    exactly on the goal.
 *
 * The goal is fixed when the command starts. If the field cannot be computed,
 * the command skips straight to [runToPose].
 *
 * ### Example
 *
 * ```kotlin
 * driverController.cross().whileTrue(navigateToPose(shootingPose))
 * ```
 *
 * @param goal The pose to drive to, in blue-origin field coordinates (flip it
 *   for the red alliance before passing it in).
 * @param entryAngle The direction to approach the goal from in the final
 *   approach, passed to [runToPose].
 * @param rotationRadius Passed to [runToPose].
 * @return The command, requiring the drive.
 */
fun navigateToPose(
    goal: Pose2d,
    entryAngle: Rotation2d? = null,
    rotationRadius: Distance? = null,
): Command =
    drive {
            val field = awaitField(goal)
            if (field != null) followField(field, goal.rotation)
            +runToPose(goal, entryAngle, rotationRadius = rotationRadius)
                .named("Navigation/FinalApproach")
        }
        .named("Navigation/NavigateToPose")

/**
 * Waits, with the drive stopped, until the goal's field is computed.
 *
 * @param goal The goal pose.
 * @return The field, or `null` if computing it failed.
 */
private fun Coroutine.awaitField(goal: Pose2d): NavField? {
    val future = navFieldCache.request(goal.translation)
    while (!future.isDone) {
        drive.stop()
        yield()
    }
    return runCatching { future.join() }.getOrNull()
}

/**
 * Follows the field downhill until the cost-to-go is below [HANDOVER_DISTANCE].
 *
 * Each loop:
 * - direction = [NavField.downhill] at the robot's position,
 * - speed = the lowest of the top speed, the braking profile on the remaining
 *   cost-to-go, and the bend limit ahead,
 * - the velocity is smoothed by [limiter] and sent with a heading PID output.
 *
 * @param field The navigation field for the goal.
 * @param heading The robot heading to turn toward, field relative.
 */
private fun Coroutine.followField(field: NavField, heading: Rotation2d) {
    headingController.update() // Pull gains from NetworkTables
    headingController.enableContinuousInput(-PI, PI)
    limiter.reset(drive.fieldOrientedSpeeds.toTranslation2d())
    val handover = HANDOVER_DISTANCE[m]

    while (true) {
        val pose = drive.pose
        val costToGo = field.costToGo(pose.translation)
        val direction = field.downhill(pose.translation)
        if (costToGo < handover || direction == null) break

        val profileSpeed = NAV_CONSTRAINTS.calculateMaxVelocity(costToGo)
        val cornerSpeed =
            field.cornerSpeedLimit(
                pose.translation,
                acceleration = NAV_CONSTRAINTS.acceleration,
                lateralAcceleration = LATERAL_ACCELERATION,
                stopBelow = handover,
            )
        val speed = minOf(NAV_CONSTRAINTS.velocity, profileSpeed, cornerSpeed)
        val velocity = limiter.calculate(direction * speed)
        val omega =
            headingController.calculate(
                pose.rotation.radians,
                heading.radians,
            )

        drive.runVelocity(
            ChassisVelocities(velocity.x.mps, velocity.y.mps, omega.rad_ps)
                .toRobotRelative(pose.rotation)
        )

        Logger.recordOutput("Navigation/CostToGo", costToGo)
        Logger.recordOutput("Navigation/Direction", direction)
        Logger.recordOutput("Navigation/SpeedLimit/Profile", profileSpeed)
        Logger.recordOutput("Navigation/SpeedLimit/Corners", cornerSpeed)
        Logger.recordOutput("Navigation/CommandedVelocity", velocity)
        yield()
    }
}
