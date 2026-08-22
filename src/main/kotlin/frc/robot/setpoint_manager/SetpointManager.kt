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
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.math.geometry.Pose2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity

object SetpointManager {
    val currentGoal: Pose2d by periodic { shootingTarget.pose }
    val calculator: SetpointCalculator by periodic {
        setpointCalculatorType.calculator
    }
    var calibrationMode = LoggedNetworkBoolean(
        "SetpointManager/calibrationMode",
        false
    )
    @LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
    var setpointCalculatorType = SetpointCalculatorType.SHOOTING
    @LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
    var shootingTarget: ShootingTarget = ShootingTarget.HUB
        set(value) {
            field = value

            setpointCalculatorType =
                when {
                    calibrationMode.get() -> SetpointCalculatorType.CALIBRATION
                    shootingTarget == ShootingTarget.HUB -> SetpointCalculatorType.SHOOTING
                    else -> SetpointCalculatorType.FEEDING
                }
        }

    @LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
    val turretSetpoint: Angle by periodic {
        calculator.calculateTurretSetpoint(
            turretOrientedChassisSpeeds,
            turretRotationToGoal,
            compensatedTurretDistanceFromGoal,
        )
    }

    @LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
    val hoodSetpoint: Angle by periodic {
        calculator.calculateHoodSetpoint(
            turretOrientedChassisSpeeds,
            compensatedTurretDistanceFromGoal,
        )
    }

    @LoggedOutput(path = "SetpointManager", level = LogLevel.COMP)
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
