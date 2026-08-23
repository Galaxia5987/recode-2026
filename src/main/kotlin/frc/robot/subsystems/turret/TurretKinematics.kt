package frc.robot.subsystems.turret

import frc.robot.drive
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.toRotation2d
import frc.robot.lib.toTranslation2d
import frc.robot.setpoint_manager.SetpointManager.currentGoal
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d by periodic {
    drive.chassisSpeeds
        .toTranslation2d()
        .plus(getTurretTangentialVelocityFieldRelative(drive.gyroOmega[rad_ps]))
        .rotateBy(Turret.motorPosition.toRotation2d())
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val turretTranslation: Translation2d by periodic {
    TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val compensatedTurretTranslation: Translation2d by periodic {
    val rotatedTurretOffset: Translation2d = drive.pose.translation
    drive.compensatedPose.translation.plus(rotatedTurretOffset)
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val angleToGoal: Rotation2d by periodic {
    turretTranslation.rotationToPoint(currentGoal.translation)
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val turretRotationToGoal: Angle by periodic {
    (drive.pose.rotation - angleToGoal).measure
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val turretDistanceFromGoal: Distance by periodic {
    turretTranslation.getDistance(currentGoal.translation).m
}

@LoggedOutput(path = "TurretKinematics", level = LogLevel.COMP)
val compensatedTurretDistanceFromGoal: Distance by periodic {
    compensatedTurretTranslation.getDistance(currentGoal.translation).m
}
