package frc.robot.field

import frc.robot.drive
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.logTrigger
import frc.robot.setpoint_manager.turretTranslation
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.Angle

const val FIELD_LOGGING_PATH = "Field"

val inClimbRectangle =
    Trigger {
            CLIMB_RECTANGLE.contains(turretTranslation)
        }
        .logTrigger("$FIELD_LOGGING_PATH/inClimbRectangle")

val inAllianceZone =
    Trigger { ALLIANCE_ZONE.contains(turretTranslation) }
        .and(inClimbRectangle.negate())
        .logTrigger("$FIELD_LOGGING_PATH/inAllianceZone")

val inExtendedAllianceZone =
    Trigger { EXTENDED_ALLIANCE_ZONE.contains(turretTranslation) }
        .and(inClimbRectangle.negate())
        .logTrigger("$FIELD_LOGGING_PATH/inExtendedAllianceZone")

val isCloserToDepotSide: Trigger =
    Trigger { OUTPOST_CROSS_LINE_RECTANGLE.contains(drive.pose.translation) }
        .negate()
        .logTrigger("$FIELD_LOGGING_PATH/isCloserToDepotSide")

val isInDoubleFeedingZone: Trigger =
    Trigger {
            DEPOT_SIDE_DOUBLE_FEEDING_RECTANGLE.contains(drive.pose.translation)
        }
        .or {
            OUTPOST_SIDE_DOUBLE_FEEDING_RECTANGLE.contains(
                drive.pose.translation
            )
        }
        .logTrigger("$FIELD_LOGGING_PATH/isInDoubleFeedingZone")

val rotationForDoubleFeeding: Angle
    get() = if (IS_RED) 0.deg else 180.deg

val ROTATION_TOLERANCE = 30.deg

val isInDoubleFeedingRotation: Trigger =
    Trigger {
            drive.pose.rotation.measure.isNear(
                rotationForDoubleFeeding,
                ROTATION_TOLERANCE,
            )
        }
        .logTrigger("$FIELD_LOGGING_PATH/isInDoubleFeedingRotation")
