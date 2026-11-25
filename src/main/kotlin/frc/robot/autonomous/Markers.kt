package frc.robot.autonomous

import frc.robot.autonomous.paths.ChoreoMarkers

/**
 * The bindings of every Choreo event marker, run in the path's coroutine when
 * the marker is reached. Awaiting a command blocks the path; forking one runs
 * it alongside the path until the path ends.
 */
object Markers : ChoreoMarkers()
