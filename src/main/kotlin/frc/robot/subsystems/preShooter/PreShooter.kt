package frc.robot.subsystems.preShooter

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

object PreShooter : Mechanism(), PreShooterVelocityCommandFactory {
    val motor =
        UniversalTalonFX(
            canbus = systemcore(0),
            port = PORT,
            gearRatio = GEAR_RATIO,
            config = CONFIG,
            logConfig = LOG_CONFIG,
            simGains = SIM_GAINS,
        )
    private var setpoint = 0.0.rps
    val atSetpoint = Trigger {
        motor.inputs.velocity.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    private val velocityVoltage = VelocityVoltage(0.0)

    init {
        addPeriodic(::periodic)
    }

    override fun setTarget(value: PreShooterVelocity) = this {
        setpoint = value.velocity
        motor.setControl(velocityVoltage.withVelocity(value.velocity))
        atSetpoint.waitUntil()
    }

    private fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/PreShooter/setpoint", setpoint)
        Logger.recordOutput("Subsystems/PreShooter/atSetpoint", atSetpoint)
    }
}
