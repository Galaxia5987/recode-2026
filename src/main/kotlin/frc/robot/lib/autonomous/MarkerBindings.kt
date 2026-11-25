package frc.robot.lib.autonomous

import org.wpilib.command3.Coroutine

/**
 * Runs the binding of a Choreo event marker by its name.
 *
 * KSP generates `ChoreoMarkers`, which implements this with one abstract
 * function per marker name across every `.traj` file. Bind them all in the
 * [frc.robot.autonomous.Markers] object.
 */
interface MarkerBindings {
    fun Coroutine.runMarker(marker: String)
}
