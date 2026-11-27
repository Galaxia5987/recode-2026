package frc.robot.subsystems.objectDetection

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.Test
import kotlin.test.assertEquals
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation3d

private const val EPSILON = 1e-9

class ObjectDetectionGeometryTest {
    private fun assertTranslationEquals(
        expected: Translation3d,
        actual: Translation3d,
    ) {
        assertEquals(expected.x, actual.x, EPSILON, "x")
        assertEquals(expected.y, actual.y, EPSILON, "y")
        assertEquals(expected.z, actual.z, EPSILON, "z")
    }

    @Test
    fun `camera pitched down sees an object ahead below its height`() {
        val pitch = PI / 6
        val robotToCamera =
            Transform3d(0.3, 0.0, 0.5, Rotation3d(0.0, pitch, 0.0))

        val robotRelative =
            Translation3d(1.0, 0.0, 0.0).toRobotRelative(robotToCamera)

        assertTranslationEquals(
            Translation3d(0.3 + cos(pitch), 0.0, 0.5 - sin(pitch)),
            robotRelative,
        )
    }

    @Test
    fun `camera yawed left maps its forward axis to robot left`() {
        val robotToCamera =
            Transform3d(0.0, 0.1, 0.4, Rotation3d(0.0, 0.0, PI / 2))

        val robotRelative =
            Translation3d(2.0, 0.0, 0.0).toRobotRelative(robotToCamera)

        assertTranslationEquals(Translation3d(0.0, 2.1, 0.4), robotRelative)
    }

    @Test
    fun `robot-relative point is placed on the field by the robot pose`() {
        val robotPose = Pose3d(Pose2d(4.0, 2.0, Rotation2d.fromRadians(PI)))

        val field = Translation3d(1.0, 0.5, 0.1).toField(robotPose)

        assertTranslationEquals(Translation3d(3.0, 1.5, 0.1), field)
    }
}
