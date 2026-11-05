package frc.robot.subsystems.gripper

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.commands.UnnamedCommand
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.hood.CANBUS
import frc.robot.subsystems.hood.ENCODER_ID
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Voltage

object Gripper : Mechanism() {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            canbus = systemcore(0),
            config= CONFIG,
            simGains= SIM_GAINS,
            logConfig =  LOG_CONFIG
        )
    private val voltageOut= VoltageOut(0.0)
private fun setVoltage(voltage: Voltage): UnnamedCommand= this {
    motor.setControl(voltageOut.withOutput(voltage))
}

    private fun periodic() {
        motor.inputs
    }

    fun intake()= setVoltage(INTAKE)
    fun outtake()= setVoltage(OUTAKE)
    fun stop()= setVoltage(STOP)


}
