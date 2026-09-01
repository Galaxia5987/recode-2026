package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.*
import frc.robot.lib.universal_motor.LoggedMotorInputs
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.math.filter.LinearFilter
import org.wpilib.system.Timer
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Voltage

object Extender : Mechanism() {
    private val io =
        when (CURRENT_MODE) {
            Mode.REAL -> ExtenderIOReal()
            Mode.SIM -> ExtenderIOSim()
            Mode.REPLAY ->
                object : ExtenderIO {
                    override val inputs: LoggedMotorInputs = LoggedMotorInputs()
                }
        }
    private val voltageOut = VoltageOut(0.0)
    private val positionVoltage = PositionVoltage(0.0)
    val inputs by PeriodicDelegate { io.inputs }
    val atSetpoint = Trigger { io.inputs.distance.isNear(setpoint, TOLERANCE) }
    var setpoint = 0.meters
        private set

    var extenderState = ExtenderState.IDLE
        private set

    init {
        addPeriodic(::periodic)
    }

    fun pump(): Command =
        this {
                while (true) {
                    +open()
                    +close()
                    yield()
                }
            }
            .whenCanceled {
                setVoltage(0.volts)
            }
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Subsystems/Extender/Pump")

    fun open(): Command =
        this {
                setpoint = OPEN_POSITION
                extenderState = ExtenderState.OPEN

                setPosition(OPEN_POSITION_ANGLE)

                atSetpoint.waitUntil()
            }
            .withPriority(Command.DEFAULT_PRIORITY)
            .named("Subsystems/Extender/Open")

    fun close(): Command =
        this {
                setVoltage(CLOSING_VOLTAGE)

                extenderState = ExtenderState.CLOSE
                setpoint = 0.meters

                val filter = LinearFilter.movingAverage(5)

                fun waitWhile(
                    condition: (velocity: AngularVelocity) -> Boolean
                ) {
                    val startTimestamp = Timer.getTimestamp()
                    while (
                        Timer.getTimestamp() - startTimestamp <
                            CLOSING_TIMEOUT[sec]
                    ) {
                        val currentVelocity =
                            filter.calculate(io.inputs.velocity[deg_ps]).deg_ps
                        Logger.recordOutput(
                            "Subsystems/Extender/filteredVelocity",
                            currentVelocity,
                        )

                        if (!condition(currentVelocity)) break
                        yield()
                    }
                }

                waitWhile { it >= CLOSING_MIN_VELOCITY }
                waitWhile { it <= CLOSING_MIN_VELOCITY }
                +stop()
            }
            .withPriority(Command.DEFAULT_PRIORITY)
            .named("Subsystems/Extender/Close")

    fun stop(): Command =
        this {
                setVoltage(0.volts)
                extenderState = ExtenderState.IDLE
            }
            .named("Subsystems/Extender/Stop")

    private fun setPosition(position: Angle) =
        io.setControl(positionVoltage.withPosition(position))

    private fun setVoltage(voltage: Voltage) =
        io.setControl(voltageOut.withOutput(voltage))

    fun periodic() {
        io.updateInputs()

        mapOf(
                "atSetpoint" to atSetpoint,
                "setpoint" to setpoint,
                "extender" to extenderState,
            )
            .forEach { (key, value) -> value.log("Subsystems/Extender", key) }
    }
}
