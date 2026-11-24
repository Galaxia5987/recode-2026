package frc.robot.lib.navigation

import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.math.tan
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.shape.Rectangle2d
import org.wpilib.units.measure.Distance

/**
 * Plans the shortest route around rectangular obstacles by going through their
 * corners.
 *
 * The idea: around a few convex obstacles, the shortest drivable route is
 * always straight lines that bend only at obstacle corners. So the only
 * waypoints needed are each obstacle's four corners, pushed outward so the
 * robot clears them. [plan] connects the start, the goal and every corner that
 * can see each other in a straight line, and picks the shortest chain with
 * Dijkstra. With a handful of obstacles that is a few dozen line checks, well
 * under a millisecond, so it can run inside a command.
 *
 * Obstacles must be axis-aligned rectangles (rotation 0° or 180°, as produced
 * by flipping), and their margins should not overlap, or the gap between them
 * is treated as closed.
 *
 * @param obstacles The obstacles, in blue-origin field coordinates.
 * @param fieldLength The field's size along x.
 * @param fieldWidth The field's size along y.
 * @param clearance How far the robot's center must stay from any obstacle when
 *   driving a straight line: its bumper radius plus a safety margin.
 * @param cornerOffset How far outside each rectangle, along both x and y, the
 *   corner waypoints sit. Must be larger than [clearance]; the extra room
 *   absorbs the corner-cutting that happens when the robot moves on to the next
 *   leg before reaching a corner.
 */
class CornerPlanner(
    obstacles: List<Rectangle2d>,
    fieldLength: Distance,
    fieldWidth: Distance,
    clearance: Distance,
    cornerOffset: Distance,
) {
    private val clearance = clearance[m]
    private val cornerOffset = cornerOffset[m]
    private val fieldLength = fieldLength[m]
    private val fieldWidth = fieldWidth[m]
    private val boxes = obstacles.map(Box::of)

    init {
        require(this.cornerOffset > this.clearance) {
            "cornerOffset must be larger than clearance"
        }
    }

    /** The corner waypoints that are inside the field and not blocked. */
    val corners: List<Translation2d> =
        boxes
            .flatMap { it.grow(this.cornerOffset).cornerPoints() }
            .filter {
                isInField(it) &&
                    boxes.none { box -> box.grow(this.clearance).contains(it) }
            }

    /**
     * Checks whether the robot can drive a straight line without hitting an
     * obstacle.
     *
     * @param from The start of the line, in meters.
     * @param to The end of the line, in meters.
     * @return `true` if the line stays at least `clearance` away from every
     *   obstacle.
     */
    fun isClear(from: Translation2d, to: Translation2d): Boolean = boxes.none {
        it.grow(clearance).intersectsSegment(from, to)
    }

    /**
     * Plans the shortest route from the start to the goal through corner
     * waypoints.
     *
     * @param start The robot's position, in meters.
     * @param goal The goal position, in meters.
     * @return The waypoints to drive through, ending with [goal] and not
     *   including [start]. Just `[goal]` when the straight line is clear.
     *   `null` if no route exists (the goal is enclosed or inside a margin).
     */
    fun plan(start: Translation2d, goal: Translation2d): List<Translation2d>? {
        // Node 0 is the start, node 1 the goal, the rest are corners.
        val nodes = listOf(start, goal) + corners
        val distance = DoubleArray(nodes.size) { Double.POSITIVE_INFINITY }
        val previous = IntArray(nodes.size) { -1 }
        val visited = BooleanArray(nodes.size)
        distance[START] = 0.0

        // Dijkstra. With a few dozen nodes a linear scan for the closest
        // unvisited node is simpler than a priority queue, and just as fast.
        while (true) {
            val current =
                nodes.indices
                    .filter { !visited[it] }
                    .minByOrNull { distance[it] }
            if (current == null || distance[current].isInfinite()) return null
            if (current == GOAL) break
            visited[current] = true
            for (next in nodes.indices) {
                if (visited[next] || !isClear(nodes[current], nodes[next])) {
                    continue
                }
                val viaCurrent =
                    distance[current] + nodes[current].getDistance(nodes[next])
                if (viaCurrent < distance[next]) {
                    distance[next] = viaCurrent
                    previous[next] = current
                }
            }
        }

        // Walk back from the goal to the start.
        return generateSequence(GOAL) { node ->
                previous[node].takeIf { it != START }
            }
            .map { nodes[it] }
            .toList()
            .reversed()
    }

    private fun isInField(point: Translation2d) =
        point.x in clearance..(fieldLength - clearance) &&
            point.y in clearance..(fieldWidth - clearance)

    /** An axis-aligned box, used for the geometry checks. */
    private data class Box(
        val minX: Double,
        val minY: Double,
        val maxX: Double,
        val maxY: Double,
    ) {
        fun grow(amount: Double) =
            Box(minX - amount, minY - amount, maxX + amount, maxY + amount)

        fun contains(point: Translation2d) =
            point.x in minX..maxX && point.y in minY..maxY

        fun cornerPoints() =
            listOf(
                Translation2d(minX, minY),
                Translation2d(maxX, minY),
                Translation2d(minX, maxY),
                Translation2d(maxX, maxY),
            )

        /**
         * Liang–Barsky line clipping: walks the segment `from + t·(to − from)`,
         * `t ∈ [0, 1]`, against each of the four box edges and narrows the
         * range of `t` that is inside the box. The segment hits the box if any
         * range is left.
         */
        fun intersectsSegment(from: Translation2d, to: Translation2d): Boolean {
            val dx = to.x - from.x
            val dy = to.y - from.y
            var tEnter = 0.0
            var tExit = 1.0
            // Each pair is (direction toward the edge, room before the edge).
            val edges =
                listOf(
                    -dx to from.x - minX,
                    dx to maxX - from.x,
                    -dy to from.y - minY,
                    dy to maxY - from.y,
                )
            for ((direction, room) in edges) {
                if (direction == 0.0) {
                    // Parallel to this edge: outside it means no hit at all.
                    if (room < 0.0) return false
                    continue
                }
                val t = room / direction
                if (direction < 0.0) tEnter = max(tEnter, t)
                else tExit = min(tExit, t)
                if (tEnter > tExit) return false
            }
            return true
        }

        companion object {
            /** Converts an axis-aligned [Rectangle2d] (0° or 180°). */
            fun of(rectangle: Rectangle2d): Box {
                require(abs(rectangle.center.rotation.sin) < 1e-6) {
                    "CornerPlanner only supports axis-aligned rectangles"
                }
                val center = rectangle.center.translation
                return Box(
                    center.x - rectangle.xWidth / 2,
                    center.y - rectangle.yWidth / 2,
                    center.x + rectangle.xWidth / 2,
                    center.y + rectangle.yWidth / 2,
                )
            }
        }
    }

    companion object {
        private const val START = 0
        private const val GOAL = 1

        /**
         * The speed to carry through a corner, so the robot only slows as much
         * as the turn needs.
         *
         * The robot rounds the corner on an arc of radius about `turnRadius /
         * tan(turn / 2)` (a sharper turn means a tighter arc). The tyres can
         * hold `v² / r ≤ lateralAcceleration`, so `v = sqrt(lateralAcceleration
         * · r)`.
         *
         * @param before The point the robot comes from, in meters.
         * @param corner The corner waypoint, in meters.
         * @param after The point the robot goes to next, in meters.
         * @param lateralAcceleration Sideways acceleration the robot can hold,
         *   in m/s².
         * @param turnRadius How far the robot cuts the corner; the corner
         *   offset is a good value, in meters.
         * @param maxSpeed The robot's top speed, in m/s.
         * @return The speed through the corner, in m/s.
         */
        fun cornerSpeed(
            before: Translation2d,
            corner: Translation2d,
            after: Translation2d,
            lateralAcceleration: Double,
            turnRadius: Double,
            maxSpeed: Double,
        ): Double {
            val incoming = corner - before
            val outgoing = after - corner
            val cosTurn =
                incoming.dot(outgoing) / (incoming.norm * outgoing.norm)
            val turn = acos(cosTurn.coerceIn(-1.0, 1.0))
            val arcRadius = turnRadius / max(tan(turn / 2), 1e-3)
            return min(maxSpeed, sqrt(lateralAcceleration * arcRadius))
        }
    }
}
