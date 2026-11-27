package frc.robot.subsystems.objectDetection

import frc.robot.lib.BetterPoseEstimator
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.extensions.toTransform
import frc.robot.lib.extensions.transformBy
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

enum class ObjectClass(val id: Long) {
    FUEL(0),
    ROBOT(1);

    companion object {
        fun fromId(id: Long) = entries.find { it.id == id }
    }
}

data class DetectedObject(
    val objectClass: ObjectClass,
    val fieldPosition: Translation3d,
    val timestamp: Double,
    private val robotRelative: Translation3d,
) {
    val distance: Double
        get() = robotRelative.norm
}

object ObjectDetection : Mechanism {
    private const val LOG_PREFIX = "Subsystems/ObjectDetection"
    private const val MAX_RESULT_AGE_SECONDS = 0.25
    private const val MATCH_RADIUS_METERS = 0.3
    private const val LOST_TIMEOUT_SECONDS = 0.5

    // Placeholder: measure on the robot. Keep the coprocessor's
    // `localization.extrinsics` at zero, or this is applied twice.
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

    var objects: List<DetectedObject> = emptyList()
        private set

    var target: DetectedObject? = null
        private set

    init {
        addPeriodic(::periodic)
    }

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
        // Read in reverse publish order, so a read that lands mid-update
        // pairs at worst one-frame-old poses with a newer timestamp.
        val classIds = classIds.get()
        val poses = cameraRelativePoses.get()
        val timestamp = timestampUs.get() / 1e6

        val age = Timer.getTimestamp() - timestamp
        if (poses.size != classIds.size || age !in 0.0..MAX_RESULT_AGE_SECONDS)
            return emptyList()

        val robotToField =
            BetterPoseEstimator.getInstance()
                .getEstimatedPoseAtTimestamp(timestamp)
                .getOrNull()
                ?.let { Pose3d(it).toTransform() } ?: return emptyList()

        return poses.zip(classIds.asList()).mapNotNull { (pose, classId) ->
            val objectClass =
                ObjectClass.fromId(classId) ?: return@mapNotNull null
            val robotRelative = pose.translation.transformBy(ROBOT_TO_CAMERA)
            DetectedObject(
                objectClass,
                robotRelative.transformBy(robotToField),
                timestamp,
                robotRelative,
            )
        }
    }

    private fun nextTarget(current: DetectedObject?): DetectedObject? {
        if (current == null || isLost(current))
            return objects.minByOrNull { it.distance }

        return objects
            .filter { it.distanceTo(current) < MATCH_RADIUS_METERS }
            .minByOrNull { it.distanceTo(current) } ?: current
    }

    private fun isLost(target: DetectedObject) =
        Timer.getTimestamp() - target.timestamp > LOST_TIMEOUT_SECONDS

    private fun DetectedObject.distanceTo(other: DetectedObject) =
        fieldPosition.getDistance(other.fieldPosition)
}
