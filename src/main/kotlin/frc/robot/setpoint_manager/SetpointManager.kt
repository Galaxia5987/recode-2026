package frc.robot.setpoint_manager

import frc.robot.drive
import frc.robot.field.HUB_TRANSLATION
import frc.robot.lib.extensions.log
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toPose
import frc.robot.lib.to2dVector
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.Units.Degrees
import org.wpilib.units.Units.Meters
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

object SetpointManager {
    private var currentGoal: Pose2d = HUB_TRANSLATION.toPose()

    val turretOrientedChassisSpeeds: Translation2d
        get() = calculateTurretVelocityVector

    private var turretTranslation: Translation2d = Translation2d(0.0, 0.0)
    private var compensatedTurretTranslation: Translation2d =
        Translation2d(0.0, 0.0)

    private var angleToGoal: Rotation2d = Rotation2d()
    private var turretRotationToGoal: Angle = Degrees.zero()

    private var turretDistanceFromGoal: Distance = Meters.zero()
    private var compensatedTurretDistanceFromGoal: Distance = Meters.zero()

    private val calculator: SetpointCalculator = GenericSetpointCalculator()

    val turretSetpoint: Angle
        get() =
            calculator.calculateTurretSetpoint(
                turretOrientedChassisSpeeds,
                turretRotationToGoal,
                compensatedTurretDistanceFromGoal,
            )

    val hoodSetpoint: Angle
        get() =
            calculator.calculateHoodSetpoint(
                turretOrientedChassisSpeeds,
                compensatedTurretDistanceFromGoal,
            )

    val flywheelSetpoint: AngularVelocity
        get() =
            calculator.calculateFlywheelSetpoint(
                turretOrientedChassisSpeeds,
                compensatedTurretDistanceFromGoal,
            )

    val calculateTurretVelocityVector: Translation2d
        get() {
            /*          val speeds = drive.chassisSpeeds
                        return speeds
                            .to2dVector()
                            .plus(
                                getTurretTangentialVelocityFieldRelative(
                                    drive.gyroOmega[rad_ps]
                                )
                            )
                            .rotateBy(Turret.position.toRotation2d())
            */

            // turret class has not been implemented yet, this is just a
            // workaround
            // todo: replace with logic above once turret is implemented
            return drive.chassisSpeeds.to2dVector()
        }

    fun periodic() {
        val rotatedTurretOffset: Translation2d = drive.pose.translation
        // todo: switch to TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)

        turretTranslation = drive.pose.translation.plus(rotatedTurretOffset)
        compensatedTurretTranslation =
            drive.compensatedPose.translation.plus(rotatedTurretOffset)

        angleToGoal = turretTranslation.rotationToPoint(currentGoal.translation)

        turretRotationToGoal = (drive.pose.rotation - angleToGoal).measure

        turretDistanceFromGoal =
            turretTranslation.getDistance(currentGoal.translation).m

        compensatedTurretDistanceFromGoal =
            compensatedTurretTranslation.getDistance(currentGoal.translation).m

        mapOf(
                "angleToGoal" to angleToGoal,
                "turretDistanceFromGoal" to turretDistanceFromGoal,
                "turretRotationToGoal" to turretRotationToGoal,
                "compensatedTurretDistanceFromGoal" to
                    compensatedTurretDistanceFromGoal,
            )
            .log("SetpointManager")
    }
}
