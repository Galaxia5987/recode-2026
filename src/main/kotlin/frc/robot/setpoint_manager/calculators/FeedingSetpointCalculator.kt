package frc.robot.setpoint_manager.calculators

import frc.robot.FeedingShotCalculator.calculateFeedingPitch
import frc.robot.FeedingShotCalculator.calculateFeedingVelocity
import frc.robot.FeedingShotCalculator.calculateFeedingYaw
import frc.robot.ShotCalculator.calculateAngularVelocity
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.setpoint_manager.SetpointCalculator
import frc.robot.setpoint_manager.accountForOvercompensation
import frc.robot.subsystems.turret.Turret.constraintTurretLimit
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

object FeedingSetpointCalculator : SetpointCalculator {
    private val MAX_FEED_VELOCITY_RPS = 42.rps

    override fun calculateTurretSetpoint(
        speeds: Translation2d,
        angleToGoal: Angle,
        distanceToGoal: Distance,
    ): Angle {
        val compensatedFeedingGoal =
            angleToGoal -
                calculateFeedingYaw(
                        distanceToGoal[m],
                        speeds.x,
                        speeds.y,
                    )
                    .deg

        return compensatedFeedingGoal.accountForOvercompensation(constraintTurretLimit(angleToGoal))
    }

    override fun calculateHoodSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): Angle {
        return (90.deg - calculateFeedingPitch().deg)
    }

    override fun calculateFlywheelSetpoint(
        speeds: Translation2d,
        distanceToGoal: Distance,
    ): AngularVelocity {
        val result =
            calculateAngularVelocity(
                    calculateFeedingVelocity(
                        distanceToGoal[m],
                        speeds.x,
                        speeds.y,
                    )
                )
                .rps

        return if (result > MAX_FEED_VELOCITY_RPS) {
            MAX_FEED_VELOCITY_RPS
        } else {
            result
        }
    }
}
