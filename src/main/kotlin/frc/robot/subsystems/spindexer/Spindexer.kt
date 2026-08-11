package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object Spindexer : Mechanism(), SpindexerVelocitiesCommandFactory {
    val motor = UniversalTalonFX(1, config = MOTOR_CONFIG, simGains = SIM_GAINS)
    val velocityRequest = VelocityVoltage(0.0)

    @LoggedOutput(LogLevel.DEV) var setpoint = 0.deg_ps

    @LoggedOutput(LogLevel.DEV)
    val atSetpoint = Trigger {
        motor.inputs.velocity.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        addPeriodic(::periodic)
    }

    override fun setTarget(value: SpindexerVelocities) = this {
        setpoint = value.velocity
        motor.setControl(velocityRequest.withVelocity(value.velocity))
        yield()
        waitUntil(atSetpoint)

        //        var i = 0
        //        println("***********************")
        //        while (true) {
        //            yield()
        //            print("${i++}: ${atSetpoint.asBoolean}, Current:
        // ${motor.inputs.velocity[rps]}, Setpoint: ${setpoint[rps]}")
        //            if (atSetpoint.asBoolean) {
        //                break
        //            }
        //        }
    }

    private fun periodic() {
        motor.periodic()
    }
}
