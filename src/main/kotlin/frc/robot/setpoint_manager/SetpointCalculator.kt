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
import kotlin.math.abs
import kotlin.math.tanh
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Distance

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

// This function checks if compensating for speed would trigger a large turn of
// the turret.
fun Angle.accountForOvercompensation(angleToGoal: Angle): Angle {
    if (abs(this[deg] - angleToGoal[deg]) > 180) return angleToGoal
    return this
}