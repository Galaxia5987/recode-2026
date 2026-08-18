package frc.robot.subsystems.funnel

import com.ctre.phoenix6.controls.VoltageOut
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Voltage

object Funnel : Mechanism() {
    private val mainMotor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG,
        )

    init {
        addPeriodic(::periodic)
    }

    private val voltageOut = VoltageOut(0.0)

    private fun setVoltage(voltage: Voltage): UnnamedCommand = this {
        mainMotor.setControl(voltageOut.withOutput(voltage))
    }

    private fun periodic() {
        Funnel.mainMotor.periodic()
    }

    fun intake(): Command =
        setVoltage(-CLOCKWISE).named("subsystems/funnel/intake")

    fun outtake(): Command =
        setVoltage(CLOCKWISE).named("subsystems/funnel/outtake")

    fun stop(): Command = setVoltage(0.0.volts).named("subsystems/funnel/stop")
}
