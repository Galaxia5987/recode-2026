package frc.robot.subsystems.intake.extender

import frc.robot.lib.extensions.meters
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism

object Extender : Mechanism()
{
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = DIAMETER,
            logConfig =
                MotorLogConfig(
                    current = false,
                    velocity = false,
                    absoluteEncoder = false,
                    controlRequest = true
                )
        )

    var atSetpoint = false
    var setpoint = 0.meters
    var extenderState = ExtenderState.IDLE

    fun pump() {

    }

    fun open() {

    }

    fun close() {

    }
}