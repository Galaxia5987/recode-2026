package frc.robot.setpoint_manager

import frc.robot.ShotCalculator.calculatePitch
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toPose
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.units.Units.Degrees
import org.wpilib.units.Units.Meters
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

// here i just added temporary field constants
// todo MOVE THIS

private val HUB_TRANSLATION_BLUE = Translation2d(4620.41.mm, 4034.63.mm)

val HUB_TRANSLATION: Translation2d
    get() = HUB_TRANSLATION_BLUE.flipIfNeeded()

//

object SetpointManager {
    var currentGoal: Pose2d = HUB_TRANSLATION.toPose()

    var turretTranslation: Translation2d = Translation2d(0.0, 0.0)
    var compensatedTurretTranslation: Translation2d = Translation2d(0.0, 0.0)

    var angleToGoal: Rotation2d = Rotation2d()
    var turretRotationToGoal: Angle = Degrees.zero()

    var turretDistanceFromGoal: Distance = Meters.zero()
    var compensatedTurretDistanceFromGoal: Distance = Meters.zero()

    val alignedTurretVelocityVector: Translation2d
        get() {
//            val speeds = drive.chassisSpeeds
//            return speeds
//                .to2dVector()
//                .plus(
//                    getTurretTangentialVelocityFieldRelative(
//                        drive.gyroOmega[rad_ps]
//                    )
//                )
//                .rotateBy(Turret.position.toRotation2d())

            // The turret class has not been implemented yet, this is just
            // a workaround.
            // todo: replace with logic above once turret is implemented
            return drive.chassisSpeeds.to2dVector()
        }

    private fun ChassisVelocities.to2dVector(): Translation2d =
        Translation2d(this.vx, this.vy)

    fun getTurretSetpoint(): Angle {
        // TODO
        return 0.degrees
    }

    private fun getHoodSetpoint(): Angle {
        val alignedTurretVelocityVector = alignedTurretVelocityVector
        return (90.deg -
                calculatePitch(
                    compensatedTurretDistanceFromGoal[m],
                    alignedTurretVelocityVector.x,
                    alignedTurretVelocityVector.y
                )
                    .deg)
    }

    fun getFlywheelSetpoint(): Angle {
        // TODO
        return 0.degrees
    }

    fun periodic() {
        val rotatedTurretOffset: Translation2d = drive.pose.translation
        // todo: switch to TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)

        turretTranslation =
            drive.pose.translation.plus(rotatedTurretOffset)
        compensatedTurretTranslation =
            drive.compensatedPose.translation.plus(rotatedTurretOffset)

        angleToGoal =
            turretTranslation.rotationToPoint(
                currentGoal.translation
            )

        turretRotationToGoal =
            (drive.pose.rotation - angleToGoal).measure

        turretDistanceFromGoal =
            turretTranslation
                .getDistance(currentGoal.translation)
                .m

        compensatedTurretDistanceFromGoal =
            compensatedTurretTranslation
                .getDistance(currentGoal.translation)
                .m
    }
}