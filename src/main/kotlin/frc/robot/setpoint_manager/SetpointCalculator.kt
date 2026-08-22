package frc.robot.setpoint_manager

import frc.robot.ShotCalculator.calculateAngularVelocity
import frc.robot.ShotCalculator.calculatePitch
import frc.robot.ShotCalculator.calculateVelocity
import frc.robot.ShotCalculator.calculateYaw
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import kotlin.math.tanh
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance
import kotlin.math.abs

interface SetpointCalculator {
    fun calculateTurretSetpoint(
        speeds: Translation2d,
        angleToGoal: Angle,
        distanceToGoal: Distance,
    ): Angle

    fun calculateHoodSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): Angle

    fun calculateFlywheelSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): AngularVelocity
}

class GenericSetpointCalculator : SetpointCalculator {
    private var kStaticCalibration = 0.97
    private var kMovingCalibration = 0.05

    override fun calculateTurretSetpoint(
        speeds: Translation2d,
        angleToGoal: Angle,
        distanceToGoal: Distance,
    ): Angle {

        val compensatedShootingGoal =
            angleToGoal -
                calculateYaw(
                        distanceToGoal[m],
                        speeds.x,
                        speeds.y,
                    )
                    .deg

        if (abs(compensatedShootingGoal[deg] - angleToGoal[deg]) > 180)
            return angleToGoal
        return compensatedShootingGoal
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
