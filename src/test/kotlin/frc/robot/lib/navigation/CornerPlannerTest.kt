package frc.robot.lib.navigation

import frc.robot.lib.extensions.m
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.shape.Rectangle2d

class CornerPlannerTest {
    /** A 1 m square block in the middle of a 10 x 6 m field. */
    private val block =
        Rectangle2d(Translation2d(4.5, 2.5), Translation2d(5.5, 3.5))

    private val planner =
        CornerPlanner(
            obstacles = listOf(block),
            fieldLength = 10.m,
            fieldWidth = 6.m,
            clearance = 0.5.m,
            cornerOffset = 0.7.m,
        )

    private val goal = Translation2d(2.0, 3.0)

    @Test
    fun `a line through the obstacle margin is not clear`() {
        assertFalse(planner.isClear(Translation2d(7.0, 3.0), goal))
        // Passes 0.4 m above the block: inside the 0.5 m clearance.
        assertFalse(
            planner.isClear(Translation2d(7.0, 3.9), Translation2d(2.0, 3.9))
        )
    }

    @Test
    fun `a line that misses the obstacle margin is clear`() {
        assertTrue(
            planner.isClear(Translation2d(7.0, 4.1), Translation2d(2.0, 4.1))
        )
        assertTrue(planner.isClear(Translation2d(2.0, 1.0), goal))
    }

    @Test
    fun `clear goal is driven to directly`() {
        assertEquals(listOf(goal), planner.plan(Translation2d(2.0, 1.0), goal))
    }

    @Test
    fun `blocked goal is reached through corners with every leg clear`() {
        val start = Translation2d(7.0, 3.0)
        val route = assertNotNull(planner.plan(start, goal))
        assertEquals(goal, route.last())
        assertTrue(route.size > 1, "expected at least one corner")
        (listOf(start) + route).zipWithNext().forEach { (from, to) ->
            assertTrue(planner.isClear(from, to), "leg $from -> $to")
        }
    }

    @Test
    fun `route is the shorter way around`() {
        // Start below the middle: going under the block is shorter.
        val route = assertNotNull(planner.plan(Translation2d(7.0, 2.9), goal))
        assertTrue(route.first().y < 2.5, "first corner ${route.first()}")
    }

    @Test
    fun `goal inside an obstacle margin has no route`() {
        assertNull(
            planner.plan(Translation2d(7.0, 3.0), Translation2d(5.0, 3.8))
        )
    }

    @Test
    fun `corners outside the field are dropped`() {
        val nearWall =
            CornerPlanner(
                obstacles =
                    listOf(
                        Rectangle2d(
                            Translation2d(4.0, 0.0),
                            Translation2d(5.0, 1.0),
                        )
                    ),
                fieldLength = 10.m,
                fieldWidth = 6.m,
                clearance = 0.5.m,
                cornerOffset = 0.7.m,
            )
        assertEquals(2, nearWall.corners.size)
        assertTrue(nearWall.corners.all { it.y > 1.0 })
    }

    @Test
    fun `corner speed is top speed on a straight and lower in sharper turns`() {
        fun speed(after: Translation2d) =
            CornerPlanner.cornerSpeed(
                before = Translation2d(0.0, 0.0),
                corner = Translation2d(1.0, 0.0),
                after = after,
                lateralAcceleration = 5.0,
                turnRadius = 0.7,
                maxSpeed = 4.0,
            )
        val straight = speed(Translation2d(2.0, 0.0))
        val right = speed(Translation2d(1.0, 1.0))
        val sharp = speed(Translation2d(0.0, 0.2))
        assertEquals(4.0, straight, 1e-9)
        assertTrue(right < straight && sharp < right, "$straight $right $sharp")
    }
}
