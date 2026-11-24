package frc.robot.lib.navigation

import frc.robot.lib.extensions.m
import kotlin.math.acos
import kotlin.math.cos
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.shape.Rectangle2d

class NavFieldTest {
    private val config =
        NavGridConfig(
            fieldLength = 10.m,
            fieldWidth = 6.m,
            cellSize = 0.05.m,
            inflation = 0.5.m,
        )

    private val goal = Translation2d(2.0, 3.0)

    /** A 1 m square block in the middle of the field, straight past goal. */
    private val block =
        Rectangle2d(Translation2d(4.5, 2.5), Translation2d(5.5, 3.5))

    private val openField = NavField.compute(NavGrid(config, emptyList()), goal)

    private val blockedField =
        NavField.compute(NavGrid(config, listOf(block.toNavObstacle())), goal)

    @Test
    fun `cost-to-go in open field is the straight-line distance`() {
        for (offset in
            listOf(
                Translation2d(3.0, 0.0),
                Translation2d(3.0, 1.5),
                Translation2d(2.0, 2.0),
            )) {
            val expected = offset.norm
            val actual = openField.costToGo(goal + offset)
            // First-order Fast Marching overestimates diagonals by ~2%.
            assertEquals(expected, actual, 0.03 * expected, "at $offset")
        }
    }

    @Test
    fun `downhill in open field points straight at the goal`() {
        val start = Translation2d(6.0, 1.0)
        val direction = assertNotNull(openField.downhill(start))
        val toGoal = (goal - start) / (goal - start).norm
        val angleError = acos(direction.dot(toGoal).coerceIn(-1.0, 1.0))
        assertTrue(angleError < Math.toRadians(3.0), "error $angleError rad")
    }

    @Test
    fun `cost-to-go behind an obstacle includes the detour`() {
        val behind = Translation2d(7.0, 3.0)
        val straightLine = behind.getDistance(goal)
        assertTrue(blockedField.costToGo(behind) > straightLine + 0.2)
    }

    @Test
    fun `following downhill goes around the obstacle and reaches the goal`() {
        val grid = blockedField.grid
        var position = Translation2d(7.0, 3.01) // Almost exactly behind it
        var minClearance = Double.POSITIVE_INFINITY
        repeat(2000) {
            val direction = blockedField.downhill(position) ?: return@repeat
            position += direction * 0.01
            minClearance = minOf(minClearance, grid.clearance(position))
        }
        assertTrue(position.getDistance(goal) < 0.1, "ended at $position")
        assertTrue(minClearance > 0.0, "entered an obstacle: $minClearance")
    }

    @Test
    fun `grid cost is infinite inside inflated obstacles and 1 far away`() {
        val grid = blockedField.grid
        val insideInflation = grid.nearestIndex(Translation2d(5.0, 3.9))
        val farAway = grid.nearestIndex(Translation2d(2.0, 3.0))
        assertEquals(Double.POSITIVE_INFINITY, grid.cost(insideInflation))
        assertEquals(1.0, grid.cost(farAway))
    }

    @Test
    fun `corner speed limit is lower before a bend than on a straight`() {
        val straight =
            openField.cornerSpeedLimit(
                Translation2d(6.0, 3.0),
                acceleration = 5.5,
                lateralAcceleration = 5.0,
                stopBelow = 0.6,
            )
        val beforeBend =
            blockedField.cornerSpeedLimit(
                Translation2d(7.0, 3.01),
                acceleration = 5.5,
                lateralAcceleration = 5.0,
                stopBelow = 0.6,
            )
        assertTrue(beforeBend < straight, "$beforeBend !< $straight")
    }
}

class TurnRateLimiterTest {
    private val limiter =
        TurnRateLimiter(acceleration = 5.0, lateralAcceleration = 5.0)

    @Test
    fun `speeding up is limited to acceleration times dt`() {
        limiter.reset(Translation2d(1.0, 0.0))
        val out = limiter.calculate(Translation2d(4.0, 0.0))
        assertEquals(1.0 + 5.0 * 0.02, out.norm, 1e-9)
    }

    @Test
    fun `braking happens immediately`() {
        limiter.reset(Translation2d(3.0, 0.0))
        val out = limiter.calculate(Translation2d(1.0, 0.0))
        assertEquals(1.0, out.norm, 1e-9)
    }

    @Test
    fun `turn rate is limited by lateral acceleration`() {
        limiter.reset(Translation2d(2.0, 0.0))
        // Goal 30 degrees to the left at the same speed.
        val goal = Translation2d(2.0, Rotation2d.fromDegrees(30.0))
        val out = limiter.calculate(goal)
        val maxTurn = 5.0 / 2.0 * 0.02 // lateral accel / speed * dt
        assertEquals(maxTurn, out.angle.get().radians, 1e-9)
        // Speed is scaled by cos(heading error) while turning.
        assertEquals(2.0 * cos(Math.toRadians(30.0)), out.norm, 1e-9)
    }

    @Test
    fun `goal behind the robot slows it down instead of orbiting`() {
        limiter.reset(Translation2d(2.0, 0.0))
        val out = limiter.calculate(Translation2d(-2.0, 0.1))
        assertEquals(0.0, out.norm, 1e-9)
    }
}
