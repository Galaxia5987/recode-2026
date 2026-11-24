package frc.robot.lib.navigation

import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import kotlin.math.roundToInt
import org.wpilib.math.geometry.Translation2d
import org.wpilib.units.measure.Distance

/**
 * Settings for building a [NavGrid].
 *
 * @property fieldLength The field's size along x.
 * @property fieldWidth The field's size along y.
 * @property cellSize The grid resolution. Smaller is more precise but slower to
 *   solve: 5 cm on a 16.5 x 8 m field is about 54k cells.
 * @property inflation How far the robot's center must stay from any obstacle
 *   surface or field wall: the robot's bumper radius plus a safety margin.
 * @property hillRange How far past the inflated edge the hill cost reaches
 *   (d₀). Past this distance the cost is back to 1.
 * @property hillWeight How expensive it is to drive right along the inflated
 *   edge (w): there one meter costs `1 + hillWeight` meters.
 */
data class NavGridConfig(
    val fieldLength: Distance,
    val fieldWidth: Distance,
    val cellSize: Distance = 0.05.m,
    val inflation: Distance,
    val hillRange: Distance = 0.5.m,
    val hillWeight: Double = 3.0,
)

/**
 * A grid over the field where every cell has a cost per meter of driving
 * through it.
 *
 * - Inside an inflated obstacle (or too close to a wall) the cost is
 *   [Double.POSITIVE_INFINITY]: the robot's center may not go there.
 * - Near an obstacle the cost rises like a hill: `1 + w·(1 − d/d₀)²`, where `d`
 *   is the distance past the inflated edge.
 * - Everywhere else the cost is 1.
 *
 * The grid depends only on the obstacles, not on the goal, so it is built once
 * and shared by every [NavField].
 *
 * Cell `(col, row)` is centered at `(col·cellSize, row·cellSize)`, so the
 * corners of the field are cell centers.
 *
 * @param config The grid settings.
 * @param obstacles The obstacles on the field. The field walls are always
 *   included and do not need to be listed.
 */
class NavGrid(config: NavGridConfig, obstacles: List<NavObstacle>) {
    /** The cell size, in meters. */
    val cellSize: Double = config.cellSize[m]

    /** The field's size along x, in meters. */
    val fieldLength: Double = config.fieldLength[m]

    /** The field's size along y, in meters. */
    val fieldWidth: Double = config.fieldWidth[m]

    /** Number of cells along x. */
    val columns: Int = (fieldLength / cellSize).roundToInt() + 1

    /** Number of cells along y. */
    val rows: Int = (fieldWidth / cellSize).roundToInt() + 1

    /** Total number of cells. */
    val size: Int
        get() = columns * rows

    private val inflation = config.inflation[m]
    private val hillRange = config.hillRange[m]
    private val hillWeight = config.hillWeight

    /** The field walls plus the given obstacles. */
    private val allObstacles: List<NavObstacle> =
        obstacles + NavObstacle(::distanceToWalls)

    /** Cost per meter of each cell, indexed by [index]. */
    private val costs = DoubleArray(size) { costAt(position(it)) }

    /**
     * @param col Column, from 0 to [columns] - 1.
     * @param row Row, from 0 to [rows] - 1.
     * @return The flat array index of the cell.
     */
    fun index(col: Int, row: Int): Int = col * rows + row

    /** @return The column of the cell with the given flat [index]. */
    fun column(index: Int): Int = index / rows

    /** @return The row of the cell with the given flat [index]. */
    fun row(index: Int): Int = index % rows

    /** @return The field position of the center of the cell, in meters. */
    fun position(index: Int): Translation2d =
        Translation2d(column(index) * cellSize, row(index) * cellSize)

    /**
     * @param point A field position, in meters.
     * @return The flat index of the cell nearest to the point, clamped to the
     *   field.
     */
    fun nearestIndex(point: Translation2d): Int =
        index(
            (point.x / cellSize).roundToInt().coerceIn(0, columns - 1),
            (point.y / cellSize).roundToInt().coerceIn(0, rows - 1),
        )

    /**
     * @param index A flat cell index.
     * @return The cell's cost per meter: 1 in open field, more near obstacles,
     *   [Double.POSITIVE_INFINITY] where the robot's center may not go.
     */
    fun cost(index: Int): Double = costs[index]

    /**
     * How far a point is past the inflated edge of the nearest obstacle or
     * wall.
     *
     * @param point A field position, in meters.
     * @return The clearance in meters. Positive means the robot's center may be
     *   there; zero or negative means the robot would hit something.
     */
    fun clearance(point: Translation2d): Double =
        allObstacles.minOf { it.distanceTo(point) } - inflation

    /** Cost per meter at a point (see the class description). */
    private fun costAt(point: Translation2d): Double {
        val d = clearance(point)
        if (d <= 0.0) return Double.POSITIVE_INFINITY
        val hill = (1.0 - d / hillRange).coerceAtLeast(0.0)
        return 1.0 + hillWeight * hill * hill
    }

    /** Distance from a point to the nearest field wall, in meters. */
    private fun distanceToWalls(point: Translation2d): Double =
        minOf(point.x, fieldLength - point.x, point.y, fieldWidth - point.y)
            .coerceAtLeast(0.0)
}
