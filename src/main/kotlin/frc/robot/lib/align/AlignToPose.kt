package frc.robot.lib.align

import frc.robot.drive
import frc.robot.lib.TuneablePIDController
import frc.robot.lib.autopilot.APConstraints
import frc.robot.lib.autopilot.APProfile
import frc.robot.lib.autopilot.APTarget
import frc.robot.lib.autopilot.Autopilot
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.extensions.radians
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import org.littletonrobotics.junction.Logger
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.LinearVelocity

private val kConstraints = APConstraints().withAcceleration(5.5).withJerk(300.0)

class AngleUnwrapper {
    private var last: Double? = null
    private var accumulated = 0.0

    fun reset() {
        last = null
    }

    fun update(wrappedRad: Double): Double {
        val prev = last
        accumulated =
            if (prev == null) wrappedRad
            else accumulated + wrap(wrappedRad - prev)
        last = wrappedRad
        return accumulated
    }

    private fun wrap(a: Double) = atan2(sin(a), cos(a))
}

private val setpointUnwrapper = AngleUnwrapper()

private val kProfile =
    APProfile(kConstraints)
        .withErrorXY(1.cm)
        .withErrorTheta(1.deg)
        .withBeelineRadius(8.cm)

private val autopilot = Autopilot(kProfile)

private val anglePIDController =
    TuneablePIDController("alignAnglePIDController", 8.0, 0.0, 0.5)

private fun Autopilot.APResult.toChassisVelocities(
    omegaResult: AngularVelocity
) =
    ChassisVelocities(
        vx,
        vy,
        omegaResult,
    )

private inline fun <T, V> T.applyIfNotNull(value: V?, block: T.(V) -> T): T =
    if (value != null) block(value) else this

fun runToPose(
    targetSupplier: () -> Pose2d,
    entryAngle: Rotation2d? = null,
    endVelocity: LinearVelocity? = null,
    rotationRadius: Distance? = null,
): UnnamedCommand = drive {
    val apTarget = {
        APTarget(targetSupplier())
            .applyIfNotNull(entryAngle) { withEntryAngle(it) }
            .applyIfNotNull(endVelocity) { withVelocity(it[mps]) }
            .applyIfNotNull(rotationRadius) { withRotationRadius(it) }
    }
    anglePIDController
        .update() // Update gains from network at the start of execution
    anglePIDController.enableContinuousInput(-PI, PI)
    setpointUnwrapper.reset()

    while (true) {
        val target = apTarget()
        val result =
            autopilot.calculate(drive.pose, drive.chassisSpeeds, target)
        val omegaResult =
            anglePIDController.calculate(
                drive.pose.rotation.radians,
                result.targetAngle.radians,
            )
        val velocities =
            result
                .toChassisVelocities(omegaResult.rad_ps)
                .toRobotRelative(drive.pose.rotation)
        val error = (result.targetAngle - drive.pose.rotation)
        val setpoint =
            setpointUnwrapper.update(result.targetAngle.radians).radians
        val measured = (setpoint[rad] - error.radians).rad
        Logger.recordOutput("Alignment/CommandedVelocities", velocities)
        Logger.recordOutput("Alignment/targetAngle", setpoint)
        Logger.recordOutput("Alignment/measuredAngle", measured)
        drive.runVelocity(velocities)
        if (autopilot.atTarget(drive.pose, target)) break
        yield()
    }
    drive.stop()
}

fun runToPose(
    target: Pose2d,
    entryAngle: Rotation2d? = null,
    endVelocity: LinearVelocity? = null,
    rotationRadius: Distance? = null,
): UnnamedCommand =
    runToPose({ target }, entryAngle, endVelocity, rotationRadius)
