package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
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

    var atSetpoint = false
    var setpoint = 0.meters
    var extenderState = ExtenderState.IDLE

    val voltageOut = VoltageOut(0.0)

    init {
        addPeriodic(::periodic)
    }

    fun pump() {

    }

    fun open() {

    }

    fun close() : Command = this {
        motor.setControl(voltageOut.withOutput(CLOSING_VOLTAGE))

        val filter = LinearFilter.movingAverage(5)
        var speed = CLOSING_MIN_VELOCITY

        while (speed >= CLOSING_MIN_VELOCITY)
        {
            speed = filter.calculate(motor.inputs.velocity[deg_ps]).deg_ps
            yield()
        }

        motor.setControl(voltageOut.withOutput(0.0))
    }.named("Extender/Close")

//    fun close() : Command = this {
//        motor.setControl(voltageOut.withOutput(CLOSING_VOLTAGE))
//
//        val velocityBuffer = ArrayDeque<Double>(5)
//        var absoluteVelocity: AngularVelocity = CLOSING_MIN_VELOCITY
//
//        Timer.getTimestamp()
//
//        velocityBuffer.forEachIndexed { index, i ->
//            velocityBuffer[index] = motor.inputs.velocity[deg_ps]
//            yield()
//        }
//
//        while (absoluteVelocity >= CLOSING_MIN_VELOCITY)
//        {
//            absoluteVelocity = calculateAbsoluteVelocity(velocityBuffer)
//
//            yield()
//        }
//
//        motor.setControl(voltageOut.withOutput(0.0))
//    }.named("CloseExtender")

//
//    private fun calculateAbsoluteVelocity(velocityBuffer: Array<Double>): AngularVelocity {
//        var angleSum: Double = 0.0
//        velocityBuffer.forEach { angle ->
//            angleSum += angle
//        }
//        return (angleSum / velocityBuffer.size).deg_ps
//    }

    fun periodic() {

    }
}