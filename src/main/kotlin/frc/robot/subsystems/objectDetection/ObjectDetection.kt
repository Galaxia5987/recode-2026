package frc.robot.subsystems.objectDetection

import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import kotlin.jvm.optionals.getOrNull
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation3d
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.system.Timer
import org.wpilib.util.Alert

data class DetectedObject(
    val classId: Int,
    /** Relative to the robot center at capture time. */
    val robotRelative: Translation3d,
    val fieldPosition: Translation3d,
    /** Capture time, seconds. */
    val timestamp: Double,
)

object ObjectDetection : Mechanism {
    private const val LOG_PREFIX = "Subsystems/ObjectDetection"
    private const val MAX_RESULT_AGE_SECONDS = 0.25
    private const val MATCH_RADIUS_METERS = 0.3
    private const val LOST_TIMEOUT_SECONDS = 0.5

    /**
     * Placeholder: measure on the robot. Keep the coprocessor's
     * `localization.extrinsics` at zero, or this is applied twice.
     */
    private val ROBOT_TO_CAMERA =
        Transform3d(300.mm, 0.mm, 500.mm, Rotation3d(0.deg, 20.deg, 0.deg))

    private val results =
        NetworkTableInstance.getDefault()
            .getTable("AetherVision")
            .getSubTable("results")
    private val timestampUs =
        results.getIntegerTopic("timestamp_us").subscribe(0)
    private val cameraRelativePoses =
        results
            .getStructArrayTopic("poses", Pose3d.struct)
            .subscribe(emptyArray())
    private val classIds =
        results.getIntegerArrayTopic("pose_class_ids").subscribe(LongArray(0))

    private val disconnectedAlert =
        Alert(
            "objectDetectionDisconnected",
            "Object detection coprocessor is disconnected.",
            Alert.Level.MEDIUM,
        )

    /** Every object in the latest fresh result; empty when stale. */
    var objects: List<DetectedObject> = emptyList()
        private set

    /**
     * The object being followed. Once locked, it only moves to a detection
     * within [MATCH_RADIUS_METERS] of its last field position, so a new object
     * coming into view can't steal it. If it goes unseen for
     * [LOST_TIMEOUT_SECONDS], the nearest object to the robot is locked next.
     */
    var target: DetectedObject? = null
        private set

    init {
        addPeriodic(::periodic)
    }

    /** Drops the lock, so the nearest object is picked on the next loop. */
    fun clearTarget() {
        target = null
    }

    private fun periodic() {
        val connected = cameraRelativePoses.topic.exists()
        disconnectedAlert.set(!connected)
        objects = if (connected) readObjects() else emptyList()
        target = nextTarget(target)

        Logger.recordOutput(
            "$LOG_PREFIX/Objects",
            *objects.map { it.fieldPosition.toPose3d() }.toTypedArray(),
        )
        Logger.recordOutput(
            "$LOG_PREFIX/Target",
            *listOfNotNull(target?.fieldPosition?.toPose3d()).toTypedArray(),
        )
    }

    private fun readObjects(): List<DetectedObject> {
        // Read the last-published topic first: a read that lands mid-update
        // then pairs at worst one-frame-old poses with a newer timestamp.
        val classIds = classIds.get()
        val poses = cameraRelativePoses.get()
        val timestamp = timestampUs.get() / 1e6

        // A negative age means the coprocessor's clock isn't synced yet.
        val age = Timer.getTimestamp() - timestamp
        if (poses.size != classIds.size || age !in 0.0..MAX_RESULT_AGE_SECONDS)
            return emptyList()

        val robotPose =
            BetterPoseEstimator.getInstance()
                .getEstimatedPoseAtTimestamp(timestamp)
                .getOrNull()
                ?.let(::Pose3d) ?: return emptyList()

        return poses.zip(classIds.asList()) { pose, classId ->
            val robotRelative =
                pose.translation.toRobotRelative(ROBOT_TO_CAMERA)
            DetectedObject(
                classId.toInt(),
                robotRelative,
                robotRelative.toField(robotPose),
                timestamp,
            )
        }
    }

    private fun nextTarget(current: DetectedObject?): DetectedObject? {
        if (current == null || isLost(current))
            return objects.minByOrNull { it.robotRelative.norm }

        return objects
            .filter {
                it.fieldPosition.getDistance(current.fieldPosition) <
                    MATCH_RADIUS_METERS
            }
            .minByOrNull { it.fieldPosition.getDistance(current.fieldPosition) }
            ?: current
    }

    private fun isLost(target: DetectedObject) =
        Timer.getTimestamp() - target.timestamp > LOST_TIMEOUT_SECONDS
}

fun Translation3d.toRobotRelative(robotToCamera: Transform3d): Translation3d =
    rotateBy(robotToCamera.rotation) + robotToCamera.translation

fun Translation3d.toField(robotPose: Pose3d): Translation3d =
    robotPose.transformBy(Transform3d(this, Rotation3d.ZERO)).translation

private fun Translation3d.toPose3d() = Pose3d(this, Rotation3d.ZERO)
