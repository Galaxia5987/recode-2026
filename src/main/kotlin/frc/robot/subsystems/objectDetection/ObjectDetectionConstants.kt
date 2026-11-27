package frc.robot.subsystems.objectDetection

import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d

/** Must match `networktables.root_table` on the coprocessor. */
const val TABLE_NAME = "AetherVision"

const val LOG_PREFIX = "Subsystems/ObjectDetection"

/**
 * Results older than this are ignored. A few frame periods: the camera stops
 * publishing, rather than publishing "no target", when it fails.
 */
const val MAX_RESULT_AGE_SECONDS = 0.25

/**
 * Camera lens relative to the robot center, WPILib convention: x forward, y
 * left, z up from the floor; positive pitch tilts the camera down.
 *
 * Placeholder values: measure them on the robot. Keep the coprocessor's
 * `localization.extrinsics` at zero, or this is applied twice.
 */
val ROBOT_TO_CAMERA =
    Transform3d(
        300.mm,
        0.mm,
        500.mm,
        Rotation3d(0.deg, 20.deg, 0.deg),
    )
