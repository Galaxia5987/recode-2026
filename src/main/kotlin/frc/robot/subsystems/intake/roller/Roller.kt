package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Voltage

object Roller : Mechanism() {
    private val mainMotor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG,
        )
    private val auxMotor =
        UniversalTalonFX(
            port = AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG,
        )

    init {
        auxMotor.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
        )
    }

    private val voltageOut = VoltageOut(0.0)

    private fun setVoltage(voltage : Voltage) : UnnamedCommand =
        this{
            mainMotor.setControl(voltageOut.withOutput(voltage))
        }

    fun intake () : Command = setVoltage(CLOCKWISE).named("Subsystems/Roller/intake")
    fun outtake () : Command = setVoltage(-CLOCKWISE).named("Subsystems/Roller/outtake")
    fun stop () : Command = setVoltage(0.0.volts).named("Subsystems/Roller/stop")

}
