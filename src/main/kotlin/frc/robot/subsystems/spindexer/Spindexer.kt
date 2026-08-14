package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object Spindexer : Mechanism(), SpindexerVelocitiesCommandFactory {
    val motor = UniversalTalonFX(1, config = MOTOR_CONFIG, simGains = SIM_GAINS)
    val velocityRequest = VelocityVoltage(0.0)

    var setpoint = 0.deg_ps
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
    }

    private fun periodic() {
        motor.periodic()
        Logger.recordOutput("subsystem/Spindexer/setpoint",setpoint)
        Logger.recordOutput("subsystem/Spindexer/atSetpoint",atSetpoint)
    }
}
