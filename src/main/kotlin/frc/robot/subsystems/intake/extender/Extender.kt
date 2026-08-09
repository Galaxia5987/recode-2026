package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.seconds
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.math.differential.Derivative
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.math.filter.LinearFilter
import org.wpilib.units.measure.AngularVelocity
import kotlin.math.abs

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
        var velocity = 0.deg_ps

        fun waitWhile(condition: (velocity: AngularVelocity) -> Boolean) {
            while (true) {
                val currentVelocity = filter.calculate(io.inputs.velocity[deg_ps]).deg_ps
                Logger.recordOutput("Subsystems/Extender/velocity", currentVelocity)

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

        Logger.recordOutput(
            "Subsystems/Extender/atSetpoint",
            atSetpoint
        )

        Logger.recordOutput(
            "Subsystems/Extender/setpoint",
            setpoint
        )

        Logger.recordOutput(
            "Subsystems/Extender/extenderState",
            extenderState
        )
    }
}