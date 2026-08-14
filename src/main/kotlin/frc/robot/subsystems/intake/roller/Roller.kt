package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.invoke
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism

object Roller : Mechanism() {
    val mainMotor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )
    val auxMotor =
        UniversalTalonFX(
            port = AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )

    init {
        auxMotor.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
        )
    }

    private val voltageOut = VoltageOut(0.0)

    fun inTake(voltageOut: VoltageOut): Command =
        this {
                mainMotor.setControl(voltageOut.withOutput(12.0))
            }
            .named("Subsystems/Roller/InTake")

    fun outTake(voltageOut: VoltageOut): Command =
        this {
                mainMotor.setControl(voltageOut.withOutput(-12.0))
            }
            .named("Subsystems/Roller/OutTake")

    fun stop(voltageOut: VoltageOut): Command =
        this {
                mainMotor.setControl(voltageOut.withOutput(0.0))
            }
            .named("Subsystems/Roller/Stop")
}
