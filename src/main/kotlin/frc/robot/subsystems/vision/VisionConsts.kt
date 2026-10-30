package frc.robot.subsystems.vision

import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation3d
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import org.wpilib.math.util.Units
import org.wpilib.vision.apriltag.AprilTagFieldLayout
import org.wpilib.vision.apriltag.AprilTagFields

private val camPitch: Double = Units.degreesToRadians(30.0)
val tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
val robotToCam =
    Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0.0, -camPitch, 0.0))
val kSingleTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(4.0, 4.0, 8.0)
val kMultiTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(0.5, 0.5, 1.0)
val transform3d =
    Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0.0, -camPitch, 0.0))
