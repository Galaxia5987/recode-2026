package frc.robot.setpoint_manager.calculators

import frc.robot.ShotCalculator.calculatePitch
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.setpoint_manager.SetpointCalculator
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

class CalibrationSetpointCalculator : SetpointCalculator {
    val calibrationVelocity = 40.0.rps

    override fun calculateTurretSetpoint(
        speeds: Translation2d,
        angleToGoal: Angle,
        distanceToGoal: Distance,
    ): Angle {
        return angleToGoal
    }

    override fun calculateHoodSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): Angle {
        return (90.deg -
                calculatePitch(
                    distanceToGoal[m],
                    speeds.x,
                    speeds.y,
                )
                    .deg)
    }

    override fun calculateFlywheelSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): AngularVelocity {
        return calibrationVelocity
    }
}
