package frc.robot.lib.navigation

import java.util.concurrent.CompletableFuture
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import org.wpilib.math.geometry.Translation2d

/**
 * Computes [NavField]s on a background thread and keeps the most recent ones.
 *
 * Solving a field takes tens of milliseconds, longer than one robot loop, so
 * commands ask for a field with [request] and wait (yielding) until it is
 * ready. Fixed goals can be computed ahead of time with [precompute], e.g. in
 * robot init, so they are ready instantly.
 *
 * Goals are keyed by position rounded to 1 cm, so a goal that is recomputed
 * every loop with tiny floating-point differences still hits the cache. Each
 * field uses about 8 bytes per cell (~430 kB on a 5 cm grid), so only the
 * [capacity] most recently used ones are kept.
 *
 * @param gridFactory Builds the shared cost grid. Called once, on the
 *   background thread, as soon as the cache is created.
 * @param capacity How many fields to keep.
 */
class NavFieldCache(
    gridFactory: () -> NavGrid,
    private val capacity: Int = 8,
) {
    private val executor: ExecutorService =
        Executors.newSingleThreadExecutor { task ->
            Thread(task, "NavFieldCache").apply { isDaemon = true }
        }

    /** The shared cost grid, built off the robot thread. */
    private val grid: CompletableFuture<NavGrid> =
        CompletableFuture.supplyAsync(gridFactory, executor)

    /** Fields by goal key (see [keyOf]), least recently used first. */
    private val fields =
        object :
            LinkedHashMap<GoalKey, CompletableFuture<NavField>>(
                capacity,
                0.75f,
                true,
            ) {
            override fun removeEldestEntry(
                eldest:
                    MutableMap.MutableEntry<
                        GoalKey,
                        CompletableFuture<NavField>,
                    >
            ) = size > capacity
        }

    /**
     * Returns the navigation field for a goal, computing it in the background
     * if it is not cached.
     *
     * @param goal The goal position, in meters.
     * @return A future that completes with the field. Already complete if the
     *   field was cached.
     */
    fun request(goal: Translation2d): CompletableFuture<NavField> =
        synchronized(fields) {
            fields.getOrPut(keyOf(goal)) {
                grid.thenApplyAsync({ NavField.compute(it, goal) }, executor)
            }
        }

    /**
     * Starts computing the fields for goals that will be needed later.
     *
     * @param goals The goal positions, in meters.
     */
    fun precompute(goals: Iterable<Translation2d>) {
        goals.forEach(::request)
    }

    /** A goal position rounded to whole centimeters. */
    private data class GoalKey(val xCm: Long, val yCm: Long)

    private fun keyOf(goal: Translation2d) =
        GoalKey(Math.round(goal.x * 100), Math.round(goal.y * 100))
}
