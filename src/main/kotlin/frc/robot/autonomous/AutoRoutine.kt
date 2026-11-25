package frc.robot.autonomous

import frc.robot.lib.commands.command
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine

internal object AutoRoutineState {
    var resetOdometryOnNextPath = false
}

private val routines = mutableListOf<Command>()

/** Every routine created with [autoRoutine], in declaration order. */
val autoRoutines: List<Command>
    get() = routines

/**
 * Creates an autonomous routine and registers it in the auto chooser. Inside
 * [block], await or fork paths and any other commands:
 * ```kotlin
 * val twoPiece =
 *     autoRoutine("Two Piece") {
 *         +StartToFuel.follow()
 *         +Shooter.shoot()
 *         +FuelToHub.follow(mirrored = true)
 *     }
 * ```
 *
 * The first path the routine follows resets odometry to its start pose. Declare
 * routines in [Autos] so they are registered on startup.
 */
fun autoRoutine(name: String, block: Coroutine.() -> Unit): Command =
    command {
            AutoRoutineState.resetOdometryOnNextPath = true
            block()
        }
        .named(name)
        .also(routines::add)
