// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision

import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.mm
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.vision.apriltag.AprilTagFieldLayout
import org.wpilib.vision.apriltag.AprilTagFields

const val LOG_PREFIX = "Subsystems/Vision/"

// AprilTag layout
val APRILTAG_LAYOUT: AprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)

// stddevFactor - Standard deviation multipliers for each camera
// (Adjust to trust some cameras more than others)
data class CameraConfig(
    val robotToCamera: () -> Transform3d,
    val botRotation: () -> Rotation2d = { drive.gyroRotation },
    val tagIdsToFilter: () -> List<Int>,
    val stddevFactor: Double,
)

// Camera names, must match names configured on coprocessor
const val MAX_DISTANCE_METERS = 5.6
val FRONT_CONFIG =
    CameraConfig(
        robotToCamera = {
            Transform3d(
                (-174.327).mm,
                (-309.901).mm,
                528.733.mm,
                Rotation3d(0.0.deg, (-20).deg, 0.deg),
            )
        },
        tagIdsToFilter = { listOf() },
        stddevFactor = 1.0,
    )

val OV_NAME_TO_CONFIG = mapOf<String, CameraConfig>("front" to FRONT_CONFIG)

// Basic filtering thresholds
const val MAX_AMBIGUITY = 0.2
const val MAX_Z_ERROR = 0.3

// Standard deviation baselines, for 1 meter distance and 1 tag
// (Adjusted automatically based on distance and # of tags)
const val LINEAR_STD_DEV_BASELINE = 0.03 // Meters
const val ANGULAR_STD_DEV_BASELINE = 0.08 // Radians[\]
