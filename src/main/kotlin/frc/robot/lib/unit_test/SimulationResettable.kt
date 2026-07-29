package frc.robot.lib.unit_test

/** State owned by a subsystem that must be cleared between simulation tests. */
/**
 * not needed for most subsystems but the ones with periodic motor control can
 * cause problems
 */
interface SimulationResettable {
    fun resetSimulationState()
}

private val simulationResettables = hashSetOf<SimulationResettable>()

fun registerSimulationResettable(resettable: SimulationResettable) {
    simulationResettables += resettable
}

fun resetRegisteredSimulationState() {
    simulationResettables.forEach { it.resetSimulationState() }
}
