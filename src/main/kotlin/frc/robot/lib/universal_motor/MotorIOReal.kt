package frc.robot.lib.universal_motor

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.lib.MotorGainTunability
import frc.robot.lib.extensions.toDistance
import frc.robot.lib.toLoggedNetworkGains
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.Distance

/**
 * Real implementation of [MotorIO] for interacting with actual TalonFX
 * hardware.
 *
 * @param port The CAN ID of the motor controller.
 * @param canBus The CAN bus name (use empty string for default).
 * @param config The TalonFX configuration to apply on startup.
 * @param gearRatio The gear ratio between motor rotations and mechanism output.
 * @param diameter The diameter of the wheel/spool if used in a linear system.
 */
class MotorIOReal(
    private val motorName: String,
    private val subsystem: String,
    private val port: Int,
    private val canBus: CANBus,
    override val config: TalonFXConfiguration,
    private val gearRatio: Double,
    private val diameter: Distance,
    private val absoluteEncoderOffset: Angle,
    private val logConfig: MotorLogConfig,
    private val logGainConfig: MotorGainTunability = MotorGainTunability(),
    ) : MotorIO {
    override val inputs = LoggedMotorInputs()
    private val motor = TalonFX(port, canBus)

    private val gains =
        config.Slot0.toLoggedNetworkGains(
            name = "$motorName/RealGains",
            subsystem = subsystem,
            motionMagicConfigs = config.MotionMagic,
            motorGainTunability = logGainConfig
        )

    init {
        motor.configurator.apply(config)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        motor.setControl(controlRequest)
    }

    override fun resetInternalEncoder(angle: Angle) {
        motor.setPosition(angle)
    }

    override fun applyConfiguration(configuration: TalonFXConfiguration) {
        motor.configurator.apply(configuration)
    }

    override fun updateInputs() {
        if (logConfig.current) inputs.current = motor.supplyCurrent.value
        if (logConfig.statorCurrent)
            inputs.statorCurrent = motor.statorCurrent.value
        if (logConfig.voltage) inputs.voltage = motor.motorVoltage.value
        if (logConfig.velocity) inputs.velocity = motor.velocity.value
        if (logConfig.absoluteEncoder)
            inputs.absoluteEncoderPositionNoOffset =
                motor.position.value - absoluteEncoderOffset
        if (logConfig.position) {
            inputs.position = motor.position.value
            inputs.distance = inputs.position.toDistance(diameter, gearRatio)
        }
        if (logConfig.controlRequest) {
            inputs.controlModeValue = motor.controlMode.value.value
        }

        if (gains.hasPIDChanged()) {
            motor.configurator.apply(gains.toSlotConfig())
        }

        if (gains.hasMotionMagicChanged()) {
            motor.configurator.apply(gains.toMotionMagicConfig())
        }
    }
}
