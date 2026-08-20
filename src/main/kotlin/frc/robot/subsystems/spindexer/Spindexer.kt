package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.commands.waitUntil
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object Spindexer : Mechanism(), SpindexerVelocitiesCommandFactory {
    private val motor =
        UniversalTalonFX(
            canbus = systemcore(0),
            port = PORT, config = MOTOR_CONFIG, simGains = SIM_GAINS
        )
    private val velocityRequest = VelocityVoltage(0.0)

    private var setpoint = 0.rps

    val atSetpoint = Trigger {
        motor.inputs.velocity.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        addPeriodic(::periodic)
    }

    override fun setTarget(value: SpindexerVelocities) = this {
        setpoint = value.velocity
        motor.setControl(velocityRequest.withVelocity(value.velocity))
        atSetpoint.waitUntil()
    }

    private fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Spindexer/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Spindexer/atSetpoint", atSetpoint)
    }
}
