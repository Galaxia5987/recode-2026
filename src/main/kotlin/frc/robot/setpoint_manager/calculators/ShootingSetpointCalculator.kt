package frc.robot.setpoint_manager.calculators

import frc.robot.ShotCalculator.calculateAngularVelocity
import frc.robot.ShotCalculator.calculatePitch
import frc.robot.ShotCalculator.calculateVelocity
import frc.robot.ShotCalculator.calculateYaw
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.setpoint_manager.SetpointCalculator
import frc.robot.setpoint_manager.accountForOvercompensation
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance
import kotlin.math.tanh

object ShootingSetpointCalculator : SetpointCalculator {
    private val kStaticCalibration = 0.97
    private val kMovingCalibration = 0.05

    override fun calculateTurretSetpoint(
        speeds: Translation2d,
        angleToGoal: Angle,
        distanceToGoal: Distance,
    ): Angle {

        val shootingGoal =
            angleToGoal -
                    calculateYaw(
                        distanceToGoal[m],
                        speeds.x,
                        speeds.y,
                    )
                        .deg

        // todo: constrain angleToGoal once turret class is implemented
        return shootingGoal.accountForOvercompensation(angleToGoal)
    }

    override fun calculateHoodSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): Angle {
        return (90 -
                calculatePitch(
                    distanceToGoal[m],
                    speeds.x,
                    speeds.y,
                ))
            .deg
    }

    override fun calculateFlywheelSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): AngularVelocity {
        return ((kStaticCalibration + (kMovingCalibration * tanh(speeds.x))) *
                calculateAngularVelocity(
                    calculateVelocity(
                        distanceToGoal[m],
                        speeds.x,
                        speeds.y,
                    )
                ))
            .rps
    }
}