package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.log
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.toAngle
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.math.filter.LinearFilter
import org.wpilib.system.Timer
import org.wpilib.units.measure.AngularVelocity

object Extender : Mechanism()
{
    private val io = ExtenderIOSim()
    private val voltageOut = VoltageOut(0.0)
    private val positionVoltage = PositionVoltage(0.0)

    val atSetpoint = Trigger { io.inputs.distance.isNear(setpoint, TOLERANCE) }
    var setpoint = 0.meters
      private set
    var extenderState = ExtenderState.IDLE
      private set

    init {
        addPeriodic(::periodic)
    }

    fun pump() : Command = this {
        try {
            while (true)
            {
                +open()
                +close()
                yield()
            }
        }
        finally
        {
            io.setControl(voltageOut.withOutput(0.0))
        }
    }.withPriority(Command.LOWEST_PRIORITY)
        .named("Subsystems/Extender/Pump")

    fun open() : Command = this {
        setpoint = OPEN_POSITION
        extenderState = ExtenderState.OPEN

        io.setControl(positionVoltage.withPosition(
            OPEN_POSITION.toAngle(DIAMETER, GEAR_RATIO))
        )

        waitUntil(atSetpoint)
    }.withPriority(Command.DEFAULT_PRIORITY)
        .named("Subsystems/Extender/Open")

    fun close() : Command = this {
        io.setControl(voltageOut.withOutput(CLOSING_VOLTAGE))
        extenderState = ExtenderState.CLOSE
        setpoint = 0.meters

        val filter = LinearFilter.movingAverage(5)
        var filterVelocity = 0.deg_ps

        fun waitWhile(condition: (velocity: AngularVelocity) -> Boolean) {
            val startTimestamp = Timer.getTimestamp()
            while (Timer.getTimestamp() - startTimestamp < CLOSING_TIMEOUT[sec]) {
                val currentVelocity = filter.calculate(io.inputs.velocity[deg_ps]).deg_ps
                Logger.recordOutput("Subsystems/Extender/filteredVelocity", currentVelocity)

                if (!condition(currentVelocity)) break
                yield()
            }
        }

        waitWhile { it >= CLOSING_MIN_VELOCITY }
        waitWhile { it <= CLOSING_MIN_VELOCITY }
        +stop()
    }.withPriority(Command.DEFAULT_PRIORITY)
        .named("Subsystems/Extender/Close")

    fun stop() : Command = this {
        io.setControl(voltageOut.withOutput(0.0))
        extenderState = ExtenderState.IDLE
    }.named("Subsystems/Extender/Stop")

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