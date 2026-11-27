package frc.robot.subsystems.objectDetection

import frc.robot.lib.extensions.AutoLogInputs
import org.wpilib.math.geometry.Pose3d

class ObjectDetectionInputs : AutoLogInputs() {
    /** The coprocessor is publishing results. */
    var connected by log(false)

    /** Camera frame number of the latest result, or -1 before the first. */
    var frame by log(NO_FRAME)

    /** When the latest result's frame was captured, robot clock, seconds. */
    var captureTimestamp by log(0.0)

    /** Capture to result finished, excluding the network hop. */
    var latencyMs by log(0.0)

    /**
     * Every object with a 3D position, best first, relative to the camera lens
     * in WPILib axes (x forward, y left, z up). Rotations are always zero:
     * depth gives position only.
     */
    var cameraRelativePoses by log(emptyArray<Pose3d>())

    /** Class id of each entry in [cameraRelativePoses]. */
    var classIds by log(LongArray(0))
}

interface ObjectDetectionIO {
    fun updateInputs(inputs: ObjectDetectionInputs) {}
}

const val NO_FRAME = -1L
