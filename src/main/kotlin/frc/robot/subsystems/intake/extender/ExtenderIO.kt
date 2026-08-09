package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.ControlRequest
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.universal_motor.LoggedMotorInputs
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX

interface ExtenderIO {
    val inputs: LoggedMotorInputs
    fun updateInputs()
    fun setControl(request: ControlRequest)
}

class ExtenderIOReal : ExtenderIO {
    private val motor = UniversalTalonFX(
        port = PORT,
        config = CONFIG,
        simGains = SIM_GAINS,
        gearRatio = GEAR_RATIO,
        linearSystemWheelDiameter = DIAMETER,
        logConfig = MotorLogConfig(
            current = false,
            velocity = true,
            absoluteEncoder = false,
            controlRequest = true
        )
    )

    override val inputs: LoggedMotorInputs
        get() = motor.inputs

    override fun updateInputs() {
        motor.periodic()
    }

    override fun setControl(request: ControlRequest) {
        motor.setControl(request)
    }
}

class ExtenderIOSim : ExtenderIO {
    private val motor = UniversalTalonFX(
        port = PORT,
        config = CONFIG,
        simGains = SIM_GAINS,
        gearRatio = GEAR_RATIO,
        linearSystemWheelDiameter = DIAMETER,
        logConfig = MotorLogConfig(
            current = false,
            velocity = true,
            absoluteEncoder = false,
            controlRequest = true
        )
    )

    override val inputs: LoggedMotorInputs
        get() = motor.inputs

    override fun updateInputs() {
        if (motor.inputs.position <= 0.deg) {
            motor.inputs.position = 0.deg
            motor.inputs.velocity = 0.deg_ps
        }

        motor.periodic()
    }

    override fun setControl(request: ControlRequest) {
        motor.setControl(request)
    }
}