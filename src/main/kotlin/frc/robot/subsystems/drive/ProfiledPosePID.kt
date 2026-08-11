package frc.robot.subsystems.drive

import frc.robot.lib.LoggedNetworkGains
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Trigger
import org.wpilib.math.controller.ProfiledPIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints

private const val LOGGING_PREFIX = "AutoAlignment"
private const val TUNING_PATH = "/Tuning/ProfiledPosePID"

@LoggedOutput(LogLevel.DISABLED) var alignmentGoal: Pose2d = Pose2d()

private val xGains = LoggedNetworkGains("X Gains", kP = 5.0, kD = 0.1)

private val yGains = LoggedNetworkGains("Y Gains", kP = 5.0, kD = 0.1)

private val thetaGains = LoggedNetworkGains("Theta Gains", 4.0, kD = 0.15)
private val linearMaxVelocity =
    LoggedNetworkNumber("$TUNING_PATH/linearMaxVelocity", 40.0)
private val linearMaxAcceleration =
    LoggedNetworkNumber("$TUNING_PATH/linearMaxAcceleration", 60.0)

private var rotationalMaxVelocity =
    LoggedNetworkNumber("$TUNING_PATH/rotationMaxVelocity", 80.0)
private var rotationalMaxAcceleration =
    LoggedNetworkNumber("$TUNING_PATH/rotationMaxAcceleration", 100.0)

private val linearLimits
    get() = Constraints(linearMaxVelocity.get(), linearMaxAcceleration.get())

private val rotationalLimits
    get() =
        Constraints(
            rotationalMaxVelocity.get(),
            rotationalMaxAcceleration.get(),
        )

@LoggedOutput(LogLevel.DISABLED, "X controller", LOGGING_PREFIX)
var xController =
    ProfiledPIDController(
        xGains.lKP.get(),
        xGains.lKI.get(),
        xGains.lKD.get(),
        linearLimits,
    )

@LoggedOutput(LogLevel.DISABLED, "Y controller", LOGGING_PREFIX)
var yController =
    ProfiledPIDController(
        yGains.lKP.get(),
        yGains.lKI.get(),
        yGains.lKD.get(),
        linearLimits,
    )

@LoggedOutput(LogLevel.DISABLED, "Theta controller", LOGGING_PREFIX)
var thetaController =
    ProfiledPIDController(
            thetaGains.lKP.get(),
            thetaGains.lKI.get(),
            thetaGains.lKD.get(),
            rotationalLimits,
        )
        .apply { enableContinuousInput(-Math.PI, Math.PI) }

@LoggedOutput(LogLevel.DISABLED, path = LOGGING_PREFIX)
var atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)

fun updateProfiledPIDGains() {
    mapOf(
            xController to Pair(xGains, linearLimits),
            yController to Pair(yGains, linearLimits),
            thetaController to Pair(thetaGains, rotationalLimits),
        )
        .forEach { (controller, pair) ->
            controller.setPID(
                pair.first.lKP.get(),
                pair.first.lKI.get(),
                pair.first.lKD.get(),
            )
            println("MAXVELOCITY ${pair.second.maxVelocity}")
            println("MAXACCEL ${pair.second.maxAcceleration}")
            controller.constraints = pair.second
        }
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPIDGains()
    alignmentGoal = desiredPose
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisVelocities) {
    xController.reset(botPose.x, botSpeeds.vx)
    yController.reset(botPose.y, botSpeeds.vy)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omega,
    )
}

fun setTolerance(pose2d: Pose2d) {
    xController.setTolerance(pose2d.x)
    yController.setTolerance(pose2d.y)
    thetaController.setTolerance(pose2d.rotation.radians)
}

/**
 * Returns field relative chassis speeds to the selected goal. @botPose the
 * current pose of the robot
 */
fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisVelocities = {
    ChassisVelocities(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians),
    )
}
