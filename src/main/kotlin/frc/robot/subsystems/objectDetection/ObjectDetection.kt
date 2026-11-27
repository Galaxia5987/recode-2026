package frc.robot.subsystems.objectDetection

import frc.robot.lib.commands.addPeriodic
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation3d
import org.wpilib.system.Timer
import org.wpilib.util.Alert

data class DetectedObject(
    val classId: Int,
    /** Relative to the robot center when the frame was captured. */
    val robotRelative: Translation3d,
    /**
     * On the field, from the robot's estimated pose at capture time. Null when
     * the pose history doesn't reach back that far.
     */
    val fieldPosition: Translation3d?,
    /** Capture time, robot clock, seconds. */
    val timestamp: Double,
)

/**
 * Objects detected by the Aether Vision coprocessor.
 *
 * [objects] is empty whenever the latest result is missing, stale or from a
 * disconnected coprocessor, so callers never act on old detections.
 *
 * @param robotToCamera Camera lens relative to the robot center.
 * @param robotPoseAt Estimated field pose of the robot at a timestamp, or null
 *   when unknown.
 */
class ObjectDetection(
    private val io: ObjectDetectionIO,
    private val robotToCamera: Transform3d,
    private val robotPoseAt: (timestamp: Double) -> Pose2d?,
) : Mechanism {
    private val inputs = ObjectDetectionInputs()
    private val disconnectedAlert =
        Alert(
            "objectDetectionDisconnected",
            "Object detection coprocessor is disconnected.",
            Alert.Level.MEDIUM,
        )

    /** Fresh detections, best first. */
    var objects: List<DetectedObject> = emptyList()
        private set

    val best: DetectedObject?
        get() = objects.firstOrNull()

    val hasTarget: Boolean
        get() = objects.isNotEmpty()

    init {
        addPeriodic(::periodic)
    }

    private fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs(LOG_PREFIX, inputs)
        disconnectedAlert.set(!inputs.connected)

        val age = Timer.getTimestamp() - inputs.captureTimestamp
        // A negative age means the coprocessor's clock isn't synced yet.
        val isFresh = inputs.connected && age in 0.0..MAX_RESULT_AGE_SECONDS
        objects = if (isFresh) detectedObjects() else emptyList()

        Logger.recordOutput("$LOG_PREFIX/IsFresh", isFresh)
        Logger.recordOutput("$LOG_PREFIX/ResultAgeSeconds", age)
        Logger.recordOutput(
            "$LOG_PREFIX/RobotRelative",
            *objects.map { it.robotRelative.toPose3d() }.toTypedArray(),
        )
        Logger.recordOutput(
            "$LOG_PREFIX/FieldPositions",
            *objects.mapNotNull { it.fieldPosition?.toPose3d() }.toTypedArray(),
        )
    }

    private fun detectedObjects(): List<DetectedObject> {
        val timestamp = inputs.captureTimestamp
        val robotPose = robotPoseAt(timestamp)?.let(::Pose3d)
        return inputs.cameraRelativePoses.zip(inputs.classIds.asList()) {
            cameraRelative,
            classId ->
            val robotRelative =
                cameraRelative.translation.toRobotRelative(robotToCamera)
            DetectedObject(
                classId = classId.toInt(),
                robotRelative = robotRelative,
                fieldPosition = robotPose?.let { robotRelative.toField(it) },
                timestamp = timestamp,
            )
        }
    }
}

/** Maps a point relative to the camera lens to the robot frame. */
fun Translation3d.toRobotRelative(robotToCamera: Transform3d): Translation3d =
    rotateBy(robotToCamera.rotation) + robotToCamera.translation

/** Maps a robot-relative point to the field, given the robot's pose. */
fun Translation3d.toField(robotPose: Pose3d): Translation3d =
    robotPose.transformBy(Transform3d(this, Rotation3d.ZERO)).translation

private fun Translation3d.toPose3d() = Pose3d(this, Rotation3d.ZERO)
