package frc.robot.setpoint_manager

import frc.robot.field.inClimbRectangle
import frc.robot.field.inExtendedAllianceZone
import frc.robot.field.isCloserToDepotSide
import frc.robot.isAuto
import frc.robot.lib.commands.command
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.periodic
import frc.robot.subsystems.turret.compensatedTurretDistanceFromGoal
import frc.robot.subsystems.turret.turretOrientedChassisSpeeds
import frc.robot.subsystems.turret.turretRotationToGoal
import org.wpilib.math.geometry.Pose2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity

object SetpointManager {
    val currentGoal: Pose2d by periodic { shootingTarget.pose }
    val calculator: SetpointCalculator by periodic {
        setpointCalculatorType.calculator
    }
    var setpointCalculatorType = SetpointCalculatorType.SHOOTING
    var shootingTarget: ShootingTarget = ShootingTarget.HUB
        set(value) {
            field = value

            if (setpointCalculatorType == SetpointCalculatorType.CALIBRATION) return

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
