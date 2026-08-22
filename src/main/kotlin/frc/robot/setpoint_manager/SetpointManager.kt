package frc.robot.setpoint_manager

import frc.robot.drive
import frc.robot.field.HUB_TRANSLATION
import frc.robot.lib.ToTranslation2d
import frc.robot.lib.extensions.CacheManager
import frc.robot.lib.extensions.log
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toPose
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

object SetpointManager {
    private val currentGoal: Pose2d by periodic { HUB_TRANSLATION.toPose() }
    private val calculator: SetpointCalculator = GenericSetpointCalculator()

    val turretOrientedChassisSpeeds: Translation2d by periodic {
        // todo: replace with full tangential velocity logic once turret class
        // is implemented
        drive.chassisSpeeds.ToTranslation2d()
    }

    private val turretTranslation: Translation2d by periodic {
        val rotatedTurretOffset: Translation2d = drive.pose.translation
        // todo: switch to TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)
        drive.pose.translation.plus(rotatedTurretOffset)
    }

    private val compensatedTurretTranslation: Translation2d by periodic {
        val rotatedTurretOffset: Translation2d = drive.pose.translation
        drive.compensatedPose.translation.plus(rotatedTurretOffset)
    }

    private val angleToGoal: Rotation2d by periodic {
        turretTranslation.rotationToPoint(currentGoal.translation)
    }

    val turretRotationToGoal: Angle by periodic {
        (drive.pose.rotation - angleToGoal).measure
    }

    val turretDistanceFromGoal: Distance by periodic {
        turretTranslation.getDistance(currentGoal.translation).m
    }

    val compensatedTurretDistanceFromGoal: Distance by periodic {
        compensatedTurretTranslation.getDistance(currentGoal.translation).m
    }

    val turretSetpoint: Angle by periodic {
        calculator.calculateTurretSetpoint(
            turretOrientedChassisSpeeds,
            turretRotationToGoal,
            compensatedTurretDistanceFromGoal,
        )
    }

    val hoodSetpoint: Angle by periodic {
        calculator.calculateHoodSetpoint(
            turretOrientedChassisSpeeds,
            compensatedTurretDistanceFromGoal,
        )
    }

    val flywheelSetpoint: AngularVelocity by periodic {
        calculator.calculateFlywheelSetpoint(
            turretOrientedChassisSpeeds,
            compensatedTurretDistanceFromGoal,
        )
    }

    fun periodic() {
        CacheManager.invalidateAll()

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
