package frc.robot.setpoint_manager

import frc.robot.drive
import frc.robot.field.inClimbRectangle
import frc.robot.field.inExtendedAllianceZone
import frc.robot.field.isCloserToDepotSide
import frc.robot.isAuto
import frc.robot.lib.ToTranslation2d
import frc.robot.lib.commands.command
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.extensions.periodic
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.setpoint_manager.SetpointManager.currentGoal
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

// TODO: MOVE ALL OF THE DEFINITIONS BELOW TO THE TURRET SUBSYSTEM

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d by periodic {
    // todo: replace with full tangential velocity logic once turret class
    // is implemented
    drive.chassisSpeeds.ToTranslation2d()
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
val turretTranslation: Translation2d by periodic {
    val rotatedTurretOffset: Translation2d = drive.pose.translation
    // todo: switch to TURRET_TO_ROBOT.rotateBy(drive.pose.rotation)
    drive.pose.translation.plus(rotatedTurretOffset)
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
private val compensatedTurretTranslation: Translation2d by periodic {
    val rotatedTurretOffset: Translation2d = drive.pose.translation
    drive.compensatedPose.translation.plus(rotatedTurretOffset)
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
private val angleToGoal: Rotation2d by periodic {
    turretTranslation.rotationToPoint(currentGoal.translation)
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
val turretRotationToGoal: Angle by periodic {
    (drive.pose.rotation - angleToGoal).measure
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
val turretDistanceFromGoal: Distance by periodic {
    turretTranslation.getDistance(currentGoal.translation).m
}

@LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
val compensatedTurretDistanceFromGoal: Distance by periodic {
    compensatedTurretTranslation.getDistance(currentGoal.translation).m
}

// TODO: MOVE ALL OF THE DEFINTIONS ABOVE TO THE TURRET SUBSYSTEM

object SetpointManager {
    private val currentGoal: Pose2d by periodic { shootingTarget.pose }
    private val calculator: SetpointCalculator by periodic {
        setpointCalculatorType.calculator
    }
    var setpointCalculatorType = SetpointCalculatorType.SHOOTING
    var shootingTarget: ShootingTarget = ShootingTarget.HUB
        set(value) {
            field = value

            setpointCalculatorType =
                when (value) {
                    ShootingTarget.HUB -> SetpointCalculatorType.SHOOTING
                    else -> SetpointCalculatorType.FEEDING
                }
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

    private val goalHubTrigger =
        inExtendedAllianceZone
            .or(isAuto)
            .onTrue(
                command {
                        shootingTarget = ShootingTarget.HUB
                    }
                    .named("SetpointManager/targetHub")
            )

    private val goalDepotTrigger =
        isCloserToDepotSide
            .and(!inExtendedAllianceZone)
            .and(inClimbRectangle.negate())
            .and(isAuto.negate())
            .onTrue(
                command {
                        shootingTarget = ShootingTarget.DEPOT
                    }
                    .named("SetpointManager/targetDepot")
            )

    private val goalOutpostTrigger =
        isCloserToDepotSide
            .negate()
            .and(!inExtendedAllianceZone)
            .and(inClimbRectangle.negate())
            .and(isAuto.negate())
            .onTrue(
                command {
                        shootingTarget = ShootingTarget.OUTPOST
                    }
                    .named("SetpointManager/targetOutpost")
            )
}
