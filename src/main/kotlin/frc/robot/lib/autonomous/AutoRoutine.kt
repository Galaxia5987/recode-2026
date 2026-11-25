package frc.robot.lib.autonomous

import frc.robot.lib.commands.command
import org.wpilib.command3.Command
import org.wpilib.command3.Coroutine

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
 *         +StartToFuel.resetOdometry()
 *         +StartToFuel.follow()
 *         +Shooter.shoot()
 *         +FuelToHub.follow(mirrored = true)
 *     }
 * ```
 *
 * Declare routines in [frc.robot.autonomous.Autos] so they are registered on
 * startup.
 */
fun autoRoutine(name: String, block: Coroutine.() -> Unit): Command =
    command(block).named(name).also(routines::add)
