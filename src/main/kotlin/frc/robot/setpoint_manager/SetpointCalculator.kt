package frc.robot.setpoint_manager

import frc.robot.field.DEPOT_TRANSLATION
import frc.robot.field.HUB_TRANSLATION
import frc.robot.field.OUTPOST_LOCATION
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.toPose
import frc.robot.setpoint_manager.calculators.CalibrationSetpointCalculator
import frc.robot.setpoint_manager.calculators.FeedingSetpointCalculator
import frc.robot.setpoint_manager.calculators.ShootingSetpointCalculator
import kotlin.math.abs
import org.wpilib.math.geometry.Pose2d
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

enum class SetpointCalculatorType(val calculator: SetpointCalculator) {
    SHOOTING(ShootingSetpointCalculator),
    FEEDING(FeedingSetpointCalculator),
    CALIBRATION(CalibrationSetpointCalculator),
}

enum class ShootingTarget {
    HUB,
    DEPOT,
    OUTPOST;

    val pose: Pose2d
        get() =
            when (this) {
                HUB -> HUB_TRANSLATION.toPose()
                DEPOT -> DEPOT_TRANSLATION.toPose()
                OUTPOST -> OUTPOST_LOCATION.toPose()
            }
}

// This function checks if compensating for speed would trigger a large turn of
// the turret.

const val OVERCOMPENSATION_THRESHOLD = 180

fun Angle.accountForOvercompensation(angleToGoal: Angle): Angle {
    if (abs(this[deg] - angleToGoal[deg]) > OVERCOMPENSATION_THRESHOLD)
        return angleToGoal
    return this
}
