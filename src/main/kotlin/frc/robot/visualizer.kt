package frc.robot

import frc.robot.lib.extensions.*
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.roller.Roller
import frc.robot.subsystems.preShooter.PreShooter
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.turret.Turret
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.*
import org.wpilib.units.measure.Angle
import kotlin.math.cos
import kotlin.math.sin

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val kWheelRadius = 0.0508.m

private fun getSwerveModulePoseTurn(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
): Pose3d {
    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw),
    )
}

private fun getSwerveModulePoseDrive(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
    modulePitch: Angle,
): Pose3d {

    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw, pitch = modulePitch),
    )
}

private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
    val swervePosesTurn: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
    for (i in 0..3) {
        swervePosesTurn[i] =
            getSwerveModulePoseTurn(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
            )
    }
    return swervePosesTurn
}

private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
    val swervePosesDrive: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

    for (i in 0..3) {
        swervePosesDrive[i] =
            getSwerveModulePoseDrive(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
                drive.SwerveDriveAngle[i],
            )
    }
    return swervePosesDrive
}

private object Intake {
    private val INTAKE_ANGLE = 9.deg
    private val EXTENDER_ANGLE_POSE by PeriodicDelegate {
        getTranslation3d(
            x = Extender.inputs.distance * cos(INTAKE_ANGLE[rad]),
            z = -Extender.inputs.distance * sin(INTAKE_ANGLE[rad]),
        )
    }

    val extender by PeriodicDelegate {
        getPose3d(getTranslation3d(80.mm, 10.mm, 248.mm) + EXTENDER_ANGLE_POSE)
    }

    val extendingHopper by PeriodicDelegate {
        getPose3d(
            getTranslation3d(346.mm, 10.mm, 235.mm) +
                    Extender.inputs.distance.toX()
        )
    }

    val roller1 by PeriodicDelegate {
        getPose3d(
            getTranslation3d(323.mm, 10.mm, 197.mm) + EXTENDER_ANGLE_POSE,
            Roller.inputs.position.toPitch(),
        )
    }

    val roller2 by PeriodicDelegate {
        getPose3d(
            getTranslation3d(260.mm, 10.mm, 197.mm) + EXTENDER_ANGLE_POSE,
            Roller.inputs.position.toPitch(),
        )
    }
}

private object Shooter {
    private val turretRotation by
    PeriodicDelegate<Rotation3d> { Turret.inputs.position.toYaw().inverse() }

    private val turretTranslation by PeriodicDelegate {
        Translation3d((-116).mm, 220.5.mm, 355.mm)
    }

    private val hoodTranslation by
    PeriodicDelegate<Translation3d> {
        getTranslation3d((-48).mm, 220.5.mm, 435.mm)
            .rotateAround(turretTranslation, turretRotation)
    }

    private val hoodRotation by
    PeriodicDelegate<Rotation3d> {
        Hood.inputs.position.toPitch().rotateBy(turretRotation)
    }

    val turret by PeriodicDelegate {
        getPose3d(turretTranslation, turretRotation)
    }

    val hood by PeriodicDelegate {
        getPose3d(hoodTranslation, hoodRotation)
    }

    val shooterMainRoller by PeriodicDelegate {
        getPose3d(
            getTranslation3d((-48).mm, 220.5.mm, 436.mm)
                .rotateAround(turretTranslation, turretRotation),
            turretRotation,
        )
            .plus(
                Transform3d(
                    Translation3d(),
                    Flywheel.inputs.position.toPitch(),
                )
            )
    }

    val hoodRoller by PeriodicDelegate {
        hood.plus(
            Transform3d(
                getTranslation3d((-247).mm, 220.5.mm, 489.mm) -
                        getTranslation3d((-48).mm, 220.5.mm, 435.mm),
                Flywheel.inputs.position.toPitch(),
            )
        )
    }
}

private object Conveyors {
    val spindexer by PeriodicDelegate {
        getPose3d(
            getTranslation3d((-26).mm, 43.mm, 36.66200.mm),
            Spindexer.inputs.position.toYaw(),
        )
    }

    val firstRoller by PeriodicDelegate {
        getPose3d(
            getTranslation3d((-63).mm, 90.mm, 211.mm),
            PreShooter.inputs.position.toRoll(),
        )
    }

    val secondRoller by PeriodicDelegate {
        getPose3d(
            getTranslation3d((-22).mm, 130.mm, 295.mm),
            PreShooter.inputs.position.toRoll(),
        )
    }

    val thirdRoller by PeriodicDelegate {
        getPose3d(
            getTranslation3d((-23.5).mm, 315.mm, 295.mm),
            PreShooter.inputs.position.toRoll(),
        )
    }
}

private object Climb {
    val grabber by PeriodicDelegate {
        getPose3d(getTranslation3d((-30).mm, (-242).mm, 420.mm))
    }
    val wrist by PeriodicDelegate {
        getPose3d(getTranslation3d((-250).mm, (-380).mm, 460.mm))
    }
}

private val subsystemPoseArray = Array(14) { Pose3d() }

@LoggedOutput(key = "Visualization/mechanismPoses", level = LogLevel.COMP)
val mechanismPoses by PeriodicDelegate {
    subsystemPoseArray[0] = Intake.extender
    subsystemPoseArray[1] = Intake.extendingHopper
    subsystemPoseArray[2] = Intake.roller1
    subsystemPoseArray[3] = Intake.roller2
    subsystemPoseArray[4] = Shooter.turret
    subsystemPoseArray[5] = Shooter.hood
    subsystemPoseArray[6] = Shooter.shooterMainRoller
    subsystemPoseArray[7] = Shooter.hoodRoller
    subsystemPoseArray[8] = Conveyors.spindexer
    subsystemPoseArray[9] = Conveyors.firstRoller
    subsystemPoseArray[10] = Conveyors.secondRoller
    subsystemPoseArray[11] = Conveyors.thirdRoller
    subsystemPoseArray[12] = Climb.grabber
    subsystemPoseArray[13] = Climb.wrist
    subsystemPoseArray
}
