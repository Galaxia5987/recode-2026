package frc.robot.subsystems.objectDetection

import org.wpilib.math.geometry.Pose3d
import org.wpilib.networktables.NetworkTableInstance

/**
 * Reads Aether Vision results from NetworkTables.
 *
 * The coprocessor's `localization.extrinsics` must be all zeros, so the
 * published poses are relative to the camera lens. The robot applies the
 * mounting transform itself (see [ObjectDetection]).
 */
class ObjectDetectionIONetworkTables(
    tableName: String = TABLE_NAME,
    instance: NetworkTableInstance = NetworkTableInstance.getDefault(),
) : ObjectDetectionIO {
    private val results = instance.getTable(tableName).getSubTable("results")

    private val timestampUs =
        results.getIntegerTopic("timestamp_us").subscribe(0)
    private val latencyMs = results.getDoubleTopic("latency_ms").subscribe(0.0)
    private val frame = results.getIntegerTopic("frame").subscribe(NO_FRAME)
    private val poses =
        results
            .getStructArrayTopic("poses", Pose3d.struct)
            .subscribe(emptyArray())
    private val classIds =
        results.getIntegerArrayTopic("pose_class_ids").subscribe(LongArray(0))

    override fun updateInputs(inputs: ObjectDetectionInputs) {
        // The topics disappear when the vision process stops or disconnects.
        inputs.connected = frame.topic.exists()

        // A result is flushed as one batch, but this read can still land
        // mid-update. Read the last-published topics first, so a split read
        // pairs at worst one-frame-old poses with a newer timestamp; it
        // corrects itself on the next loop.
        val classIds = classIds.get()
        val poses = poses.get()
        if (poses.size != classIds.size) return

        inputs.classIds = classIds
        inputs.cameraRelativePoses = poses
        inputs.frame = frame.get()
        inputs.latencyMs = latencyMs.get()
        inputs.captureTimestamp = timestampUs.get() / 1e6
    }
}
