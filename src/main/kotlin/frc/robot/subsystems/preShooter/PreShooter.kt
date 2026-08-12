package frc.robot.subsystems.preShooter

import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger

object PreShooter : Mechanism(), PreShooerVelocityCommandFactory {
    val motor = UniversalTalonFX(port = 1)
    val ControlRequest = VelocityVoltage(0.0)
    var setpoint = 0.0.rps
    var atSetpoint = Trigger {
        motor.inputs.velocity.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        addPeriodic(::periodic)
    }

    override fun setTarget(value: PreShooerVelocity) = this {
        setpoint = value.velocity
        motor.setControl(ControlRequest.withVelocity(value.velocity))
        waitUntil(atSetpoint)
    }

    private fun periodic() {
        motor.periodic()
        Logger.recordOutput("subsystem/$name/setpoint", setpoint)
        Logger.recordOutput("subsystem/$name/atSetpoint", atSetpoint)
    }
}
