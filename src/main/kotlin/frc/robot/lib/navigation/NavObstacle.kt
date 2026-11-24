package frc.robot.lib.navigation

import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.shape.Rectangle2d

/**
 * A static obstacle on the field that navigation keeps the robot away from.
 *
 * Obstacles are described by their real (un-inflated) shape. [NavGrid] adds the
 * robot's size and a safety margin on top.
 */
fun interface NavObstacle {
    /**
     * Distance from a point to the obstacle's surface.
     *
     * @param point A field position, in meters.
     * @return The distance to the closest point of the obstacle, in meters. 0
     *   when the point is on or inside the obstacle.
     */
    fun distanceTo(point: Translation2d): Double
}

/**
 * Wraps a [Rectangle2d] as an obstacle. The rectangle may be rotated.
 *
 * @return An obstacle whose distance is [Rectangle2d.getDistance].
 */
fun Rectangle2d.toNavObstacle(): NavObstacle = NavObstacle(::getDistance)

/**
 * A round obstacle.
 *
 * @param center The circle's center, in meters.
 * @param radius The circle's radius, in meters.
 * @return An obstacle whose distance is measured to the circle's edge.
 */
fun circleNavObstacle(center: Translation2d, radius: Double): NavObstacle =
    NavObstacle { point ->
        (point.getDistance(center) - radius).coerceAtLeast(0.0)
    }
