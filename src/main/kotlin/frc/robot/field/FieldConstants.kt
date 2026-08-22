package frc.robot.field

import com.pathplanner.lib.util.FlippingUtil
import frc.robot.lib.extensions.flipIfNeeded
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mirror
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.periodic
import org.wpilib.math.geometry.Rectangle2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m
val EXTENDED_ALLIANCE_ZONE_WIDTH: Distance = 5.237.m

private val HUB_TRANSLATION_BLUE = Translation2d(4620.41.mm, 4034.63.mm)

val HUB_TRANSLATION: Translation2d by periodic {
    HUB_TRANSLATION_BLUE.flipIfNeeded()
}

private val OUTPOST_FEED_TRANSLATION = Translation2d(2.607.m, 1.6.m)

val OUTPOST_LOCATION: Translation2d
    get() = OUTPOST_FEED_TRANSLATION.flipIfNeeded()

private val DEPOT_TRANSLATION_BLUE =
    Translation2d(
        OUTPOST_FEED_TRANSLATION.measureX,
        FlippingUtil.fieldSizeY.m - OUTPOST_FEED_TRANSLATION.measureY,
    )

val DEPOT_TRANSLATION: Translation2d
    get() = DEPOT_TRANSLATION_BLUE.flipIfNeeded()

private val ALLIANCE_ZONE_BLUE_RECTANGLE =
    Rectangle2d(
        Translation2d(0.m, Int.MIN_VALUE.m),
        Translation2d(ALLIANCE_ZONE_WIDTH, Int.MAX_VALUE.m),
    )

private val EXTENDED_ALLIANCE_ZONE_BLUE_RECTANGLE =
    Rectangle2d(
        Translation2d(0.m, Int.MIN_VALUE.m),
        Translation2d(EXTENDED_ALLIANCE_ZONE_WIDTH, Int.MAX_VALUE.m),
    )

val ALLIANCE_ZONE: Rectangle2d
    get() = ALLIANCE_ZONE_BLUE_RECTANGLE.flipIfNeeded()

val EXTENDED_ALLIANCE_ZONE: Rectangle2d
    get() = EXTENDED_ALLIANCE_ZONE_BLUE_RECTANGLE.flipIfNeeded()

private val CLIMB_RECTANGLE_BLUE =
    Rectangle2d(Translation2d(0.0, 3.113), Translation2d(1.060, 4.363))

val CLIMB_RECTANGLE: Rectangle2d
    get() = CLIMB_RECTANGLE_BLUE.flipIfNeeded()

private val OUTPOST_CROSS_LINE_BLUE_RECTANGLE =
    Rectangle2d(
        Translation2d(0.m, Int.MIN_VALUE.m),
        Translation2d(FlippingUtil.fieldSizeX, FlippingUtil.fieldSizeY / 2),
    )

val OUTPOST_CROSS_LINE_RECTANGLE: Rectangle2d
    get() = OUTPOST_CROSS_LINE_BLUE_RECTANGLE.flipIfNeeded()

private val DEPOT_SIDE_DOUBLE_FEEDING_RECTANGLE_BLUE: Rectangle2d =
    Rectangle2d(Translation2d(7.852, 1.1), Translation2d(4.05, 0.2))

private val OUTPOST_SIDE_DOUBLE_FEEDING_RECTANGLE_BLUE: Rectangle2d =
    DEPOT_SIDE_DOUBLE_FEEDING_RECTANGLE_BLUE.mirror()

val DEPOT_SIDE_DOUBLE_FEEDING_RECTANGLE: Rectangle2d
    get() = DEPOT_SIDE_DOUBLE_FEEDING_RECTANGLE_BLUE.flipIfNeeded()

val OUTPOST_SIDE_DOUBLE_FEEDING_RECTANGLE: Rectangle2d
    get() = OUTPOST_SIDE_DOUBLE_FEEDING_RECTANGLE_BLUE.flipIfNeeded()
