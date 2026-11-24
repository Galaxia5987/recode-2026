package frc.robot.lib.navigation

import java.util.PriorityQueue
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt
import org.wpilib.math.geometry.Translation2d

/**
 * A navigation field: for every point on the field, how far the robot still has
 * to drive to reach one goal, going around obstacles.
 *
 * The value is called the cost-to-go `T`. It is in cost-weighted meters: in
 * open field it is the real driving distance, and near obstacles every meter
 * counts as more, according to the [NavGrid] hills.
 *
 * `T` has exactly one minimum, at the goal, so driving downhill (along `−∇T`,
 * see [downhill]) always follows the cheapest route to the goal and can never
 * get stuck, unlike a classic potential field.
 *
 * Build one with [compute]. A field is immutable, so it can be computed on a
 * background thread and then read from the robot loop.
 *
 * @property grid The cost grid the field was computed on.
 * @property goal The goal position, in meters.
 */
class NavField
private constructor(
    val grid: NavGrid,
    val goal: Translation2d,
    /** `T` per cell, with blocked/unreachable cells raised above the rest. */
    private val costToGo: DoubleArray,
) {
    /**
     * The cost-to-go at a point, interpolated between the four surrounding
     * cells (bilinear interpolation).
     *
     * @param point A field position, in meters. Points outside the field are
     *   clamped to its edge.
     * @return The cost-weighted distance left to the goal, in meters.
     */
    fun costToGo(point: Translation2d): Double {
        val fx = (point.x / grid.cellSize).coerceIn(0.0, grid.columns - 1.0)
        val fy = (point.y / grid.cellSize).coerceIn(0.0, grid.rows - 1.0)
        // Lower-left cell of the square around the point.
        val col = min(fx.toInt(), grid.columns - 2)
        val row = min(fy.toInt(), grid.rows - 2)
        val u = fx - col
        val v = fy - row
        return (1 - u) * (1 - v) * costToGo[grid.index(col, row)] +
            u * (1 - v) * costToGo[grid.index(col + 1, row)] +
            (1 - u) * v * costToGo[grid.index(col, row + 1)] +
            u * v * costToGo[grid.index(col + 1, row + 1)]
    }

    /**
     * The direction to drive from a point: straight downhill on `T`.
     *
     * Computed by comparing `T` one cell to each side of the point (central
     * differences), so it changes smoothly as the robot moves.
     *
     * @param point A field position, in meters.
     * @return A unit vector along `−∇T`, or `null` where `T` is flat (at the
     *   goal itself).
     */
    fun downhill(point: Translation2d): Translation2d? {
        val h = grid.cellSize
        val dx = Translation2d(h, 0.0)
        val dy = Translation2d(0.0, h)
        val gradient =
            Translation2d(
                costToGo(point + dx) - costToGo(point - dx),
                costToGo(point + dy) - costToGo(point - dy),
            ) / (2 * h)
        val norm = gradient.norm
        return if (norm < 1e-9) null else -gradient / norm
    }

    /**
     * The fastest speed from which the robot can still slow down enough for
     * every bend in the route ahead.
     *
     * Walks the downhill route forward from the point in small steps. At each
     * step it measures how sharply the direction turns (the bend radius `R =
     * step / turn angle`). The tyres can only hold `v² / R ≤
     * lateralAcceleration`, so the bend allows `sqrt(lateralAcceleration · R)`.
     * Being `s` meters before that bend, the robot may go `sqrt(vBend² + 2 ·
     * acceleration · s)` and still brake in time.
     *
     * @param point The robot's position, in meters.
     * @param acceleration The braking the robot can use, in m/s².
     * @param lateralAcceleration The sideways acceleration the robot can hold
     *   in a bend, in m/s².
     * @param stopBelow Stop looking ahead once `T` drops below this, in meters
     *   (the final approach is handled by Autopilot).
     * @param step The look-ahead step length, in meters.
     * @param horizon How far to look ahead, in meters. Should cover the braking
     *   distance from top speed.
     * @return The speed limit in m/s, [Double.POSITIVE_INFINITY] if the route
     *   ahead is straight.
     */
    fun cornerSpeedLimit(
        point: Translation2d,
        acceleration: Double,
        lateralAcceleration: Double,
        stopBelow: Double,
        step: Double = 0.1,
        horizon: Double = 3.0,
    ): Double {
        var limit = Double.POSITIVE_INFINITY
        var position = point
        var direction = downhill(position) ?: return limit
        var travelled = 0.0
        while (travelled < horizon && costToGo(position) > stopBelow) {
            position += direction * step
            travelled += step
            val next = downhill(position) ?: break
            val turn = acos(direction.dot(next).coerceIn(-1.0, 1.0))
            if (turn > 1e-6) {
                val bendSpeed = sqrt(lateralAcceleration * step / turn)
                limit =
                    min(
                        limit,
                        sqrt(
                            bendSpeed * bendSpeed + 2 * acceleration * travelled
                        ),
                    )
            }
            direction = next
        }
        return limit
    }

    companion object {
        /**
         * How much higher than the farthest reachable cell blocked and
         * unreachable cells are set, in meters. This keeps the interpolation
         * finite and makes the slope point out of obstacles.
         */
        private const val BLOCKED_PENALTY = 5.0

        /**
         * Computes the navigation field for a goal with the Fast Marching
         * Method.
         *
         * Fast Marching spreads outward from the goal like a wave front, fast
         * through cheap cells and slowly through hills, and records when the
         * front reaches each cell. Cells are finalized in order of increasing
         * `T`, so each is solved once: about `N log N` for `N` cells. Unlike
         * Dijkstra on grid moves, each update combines a horizontal and a
         * vertical neighbour, which models driving at any angle (distances come
         * out within ~2% of the true straight-line distance instead of up to
         * 8–40% off).
         *
         * Takes tens of milliseconds for a 5 cm grid, so call it off the robot
         * loop (see [NavFieldCache]).
         *
         * @param grid The cost grid.
         * @param goal The goal position, in meters. It should be in a free
         *   cell; if it is inside an inflated obstacle, the field is only
         *   defined where free cells touch the goal cell.
         * @return The navigation field for the goal.
         */
        fun compute(grid: NavGrid, goal: Translation2d): NavField {
            val costToGo = solveEikonal(grid, grid.nearestIndex(goal))
            val maxReachable =
                costToGo.filter { it.isFinite() }.maxOrNull() ?: 0.0
            for (i in costToGo.indices) {
                if (!costToGo[i].isFinite()) {
                    costToGo[i] = maxReachable + BLOCKED_PENALTY
                }
            }
            return NavField(grid, goal, costToGo)
        }

        /**
         * Solves `|∇T| = cost` outward from one cell (first-order Fast
         * Marching).
         *
         * @param grid The cost grid.
         * @param goalIndex The flat index of the goal cell.
         * @return `T` per cell, in meters; [Double.POSITIVE_INFINITY] for
         *   blocked and unreachable cells.
         */
        private fun solveEikonal(grid: NavGrid, goalIndex: Int): DoubleArray {
            val costToGo = DoubleArray(grid.size) { Double.POSITIVE_INFINITY }
            val finalized = BooleanArray(grid.size)
            val front = PriorityQueue<FrontCell>()

            costToGo[goalIndex] = 0.0
            front.add(FrontCell(0.0, goalIndex))

            while (front.isNotEmpty()) {
                val cell = front.poll().index
                // A cell can be queued several times; only its first (lowest)
                // entry counts.
                if (finalized[cell]) continue
                finalized[cell] = true

                grid.forEachNeighbour(cell) { neighbour ->
                    if (finalized[neighbour]) return@forEachNeighbour
                    if (!grid.cost(neighbour).isFinite()) {
                        return@forEachNeighbour
                    }
                    val candidate =
                        grid.localUpdate(neighbour, costToGo, finalized)
                    if (candidate < costToGo[neighbour]) {
                        costToGo[neighbour] = candidate
                        front.add(FrontCell(candidate, neighbour))
                    }
                }
            }
            return costToGo
        }

        /**
         * The Fast Marching update for one cell.
         *
         * Takes the smallest finalized `T` among its left/right neighbours
         * (`a`) and among its up/down neighbours (`b`), and solves for the time
         * a straight wave front, coming from whichever direction fits both,
         * reaches the cell: `(T − a)² + (T − b)² = (cost · cellSize)²`. If only
         * one side is close enough to matter, `T = min(a, b) + cost ·
         * cellSize`.
         *
         * @param index The cell to update.
         * @param costToGo The current `T` values.
         * @param finalized Which cells are finalized.
         * @return The new candidate `T` for the cell, in meters.
         */
        private fun NavGrid.localUpdate(
            index: Int,
            costToGo: DoubleArray,
            finalized: BooleanArray,
        ): Double {
            val col = column(index)
            val row = row(index)
            fun known(c: Int, r: Int): Double {
                if (c !in 0 until columns || r !in 0 until rows) {
                    return Double.POSITIVE_INFINITY
                }
                val i = index(c, r)
                return if (finalized[i]) costToGo[i]
                else Double.POSITIVE_INFINITY
            }
            val a = min(known(col - 1, row), known(col + 1, row))
            val b = min(known(col, row - 1), known(col, row + 1))
            val step = cost(index) * cellSize
            return if (kotlin.math.abs(a - b) >= step) {
                min(a, b) + step
            } else {
                0.5 * (a + b + sqrt(2 * step * step - (a - b) * (a - b)))
            }
        }

        /**
         * Calls [action] with each of the up to 4 side neighbours of a cell.
         */
        private inline fun NavGrid.forEachNeighbour(
            index: Int,
            action: (Int) -> Unit,
        ) {
            val col = column(index)
            val row = row(index)
            if (col > 0) action(index(col - 1, row))
            if (col < columns - 1) action(index(col + 1, row))
            if (row > 0) action(index(col, row - 1))
            if (row < rows - 1) action(index(col, row + 1))
        }
    }

    /** A cell waiting on the Fast Marching front, ordered by its `T`. */
    private data class FrontCell(val costToGo: Double, val index: Int) :
        Comparable<FrontCell> {
        override fun compareTo(other: FrontCell) =
            costToGo.compareTo(other.costToGo)
    }
}
