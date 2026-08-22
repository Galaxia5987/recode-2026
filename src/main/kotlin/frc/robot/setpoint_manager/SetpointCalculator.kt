package frc.robot.setpoint_manager

import frc.robot.FeedingShotCalculator.calculateFeedingPitch
import frc.robot.FeedingShotCalculator.calculateFeedingVelocity
import frc.robot.FeedingShotCalculator.calculateFeedingYaw
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

// This function checks if compensating for speed would trigger a large turn of the turret.
fun Angle.accountForOvercompensation(angleToGoal: Angle): Angle {
    if (abs(this[deg] - angleToGoal[deg]) > 180)
        return angleToGoal
    return this
}

class GenericSetpointCalculator : SetpointCalculator {
    private var kStaticCalibration = 0.97
    private var kMovingCalibration = 0.05

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

class FeedingSetpointCalculator : SetpointCalculator {
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

        // todo: constrain angleToGoal once turret class is implemented
        return compensatedFeedingGoal.accountForOvercompensation(angleToGoal)
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
            ).rps

        return if (result > MAX_FEED_VELOCITY_RPS) {
            MAX_FEED_VELOCITY_RPS
        } else {
            result
        }
    }
}
