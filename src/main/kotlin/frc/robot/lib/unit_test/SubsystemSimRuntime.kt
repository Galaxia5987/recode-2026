package frc.robot.lib.unit_test

import edu.wpi.first.hal.HAL
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.junit.jupiter.api.AfterEach

private const val PERIODIC_TIME: Double = 0.02

val allMotorsFromPorts = hashMapOf<Pair<Int, String>, UniversalTalonFX>()

fun getInputs(port: Int, canbus: String = "rio"): LoggedMotorInputs =
    allMotorsFromPorts[port to canbus]!!.inputs

object SubsystemSimRuntime {
    val scheduler: CommandScheduler = CommandScheduler.getInstance()

    fun addCommands(vararg commands: Command): SubsystemSimRuntime {
        commands.forEach {
            it.requirements.forEach { subsystem ->
                if (subsystem is SimulationResettable) {
                    registerSimulationResettable(subsystem)
                }
            }
        }
        scheduler.schedule(*commands)
        return this
    }

    init {
        check(HAL.initialize(500, 0))
        SimHooks.pauseTiming()

        DriverStationSim.setEnabled(true)
        DriverStationSim.notifyNewData()
    }

    fun step() {
        SimHooks.stepTiming(PERIODIC_TIME)
        scheduler.run()
    }

    fun runFor(seconds: Time) {
        repeat((seconds[sec] / PERIODIC_TIME).toInt()) { step() }
        DriverStationSim.notifyNewData()
        reset()
    }

    fun runUntil(timeoutSeconds: Time, condition: () -> Boolean): Boolean {
        repeat((timeoutSeconds[sec] / PERIODIC_TIME).toInt()) {
            step()
            if (condition()) {
                DriverStationSim.notifyNewData()
                reset()
                return true
            }
        }
        DriverStationSim.notifyNewData()
        reset()
        return false
    }

    fun reset() {
        scheduler.cancelAll()
        scheduler.clearComposedCommands()
    }

    fun restartInputs() {
        reset()
        resetRegisteredSimulationState()
        allMotorsFromPorts.values.distinct().forEach { it.resetInputs() }
    }
}

abstract class Tests {
    @AfterEach
    fun `clean inputs`() {
        SubsystemSimRuntime.restartInputs()
    }
}
