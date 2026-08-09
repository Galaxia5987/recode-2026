package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.unaryPlus
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.math.filter.LinearFilter

object Extender : Mechanism()
{
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = DIAMETER,
            logConfig =
                MotorLogConfig(
                    current = false,
                    velocity = false,
                    absoluteEncoder = false,
                    controlRequest = true
                )
        )

    val atSetpoint = Trigger { motor.inputs.distance.isNear(setpoint, TOLERANCE) }
    var setpoint = 0.meters
    var extenderState = ExtenderState.IDLE

    val voltageOut = VoltageOut(0.0)
    val positionVoltage = PositionVoltage(0.0)

    init {
        addPeriodic(::periodic)
    }

    fun pump() : Command = this {
        while (true)
        {
            +open()
            +close()
            yield()
        }
    }.named("Extender/Pump")

    fun open() : Command = this {
        setpoint = OPEN_POSITION
        extenderState = ExtenderState.OPEN

        motor.setControl(positionVoltage.withPosition(
            OPEN_POSITION.toAngle(DIAMETER, GEAR_RATIO))
        )

        waitUntil { atSetpoint.asBoolean }
    }.named("Extender/Open")

    fun close() : Command = this {
        motor.setControl(voltageOut.withOutput(CLOSING_VOLTAGE))
        extenderState = ExtenderState.CLOSE
        setpoint = 0.meters

        val filter = LinearFilter.movingAverage(5)
        var speed = CLOSING_MIN_VELOCITY

        while (speed >= CLOSING_MIN_VELOCITY)
        {
            speed = filter.calculate(motor.inputs.velocity[deg_ps]).deg_ps
            yield()
        }

        +stop()
    }.named("Extender/Close")

    fun stop() : Command = this {
        motor.setControl(voltageOut.withOutput(0.0))
        extenderState = ExtenderState.IDLE
    }.named("Extender/Stop")

    fun periodic() {

    }
}