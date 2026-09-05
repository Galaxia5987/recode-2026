package frc.robot.subsystems.intake.funnel

import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.with
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism

object Funnel : Mechanism(), FunnelStateCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG,
        )

    init {
        addPeriodic(mainMotor::periodic)
    }

    private val voltageOut = VoltageOut(0.0)

    override fun setTarget(value: FunnelState): UnnamedCommand = this {
        mainMotor.setControl(voltageOut with value.voltage)
    }
}
