package frc.robot.lib.universal_motor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import frc.robot.lib.Gains
import frc.robot.lib.GainsEnum
import frc.robot.lib.extensions.*
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toNetworkLogged
import org.wpilib.math.controller.PIDController
import org.wpilib.math.controller.ProfiledPIDController
import org.wpilib.math.trajectory.TrapezoidProfile
import org.wpilib.system.Timer
import org.wpilib.units.measure.Distance
import org.wpilib.units.measure.MomentOfInertia

/**
 * Simulated implementation of [MotorIO] for use during robot simulation.
 *
 * @param momentOfInertia The moment of inertia of the simulated mechanism.
 * @param config The TalonFX configuration used to build the PID controller.
 * @param gearRatio The gear ratio between motor output and mechanism.
 * @param diameter The wheel/spool diameter for computing linear distance.
 */
class MotorIOSim(
    private val motorName: String,
    private val subsystem: String,
    private val momentOfInertia: MomentOfInertia,
    override val config: TalonFXConfiguration,
    private val simGains: Gains,
    private val gearRatio: Double,
    private val diameter: Distance,
    private val logConfig: MotorLogConfig,
) : MotorIO {
    private val gains =
        simGains.toNetworkLogged(
            name = "$motorName/SimGains",
            subsystem = subsystem,
            motionMagicConfigs = config.MotionMagic,
        )

    override val inputs = LoggedMotorInputs()
    private val profiledPIDController =
        ProfiledPIDController(
            simGains.kP,
            simGains.kI,
            simGains.kD,
            TrapezoidProfile.Constraints(
                config.MotionMagic.MotionMagicCruiseVelocity,
                config.MotionMagic.MotionMagicAcceleration,
            ),
        )
    private val controller =
        PIDController(simGains.kP, simGains.kI, simGains.kD).apply {
            if (config.ClosedLoopGeneral.ContinuousWrap) {
                enableContinuousInput(0.0, 1.0)
            }
        }
    private val motor =
        TalonFXSim(1, 1.0, momentOfInertia[kg2m], 1.0, TalonType.KRAKEN_FOC)

    init {
        motor.setController(controller)
        motor.setProfiledController(profiledPIDController)
    }

    override fun setRequest(controlRequest: ControlRequest) {
        when (controlRequest) {
            is VelocityVoltage ->
                controlRequest.FeedForward =
                    controlRequest.Velocity * simGains.kV

            is VelocityTorqueCurrentFOC ->
                controlRequest.FeedForward =
                    controlRequest.Velocity * simGains.kV

            is PositionVoltage ->
                controlRequest.FeedForward =
                    controlRequest.Position * simGains.kV

            is PositionTorqueCurrentFOC ->
                controlRequest.FeedForward =
                    controlRequest.Position * simGains.kV
        }

        motor.setControl(controlRequest)
    }

    override fun resetInputs() {
        motor.resetInputs()
        inputs.position = 0.deg
        inputs.distance = 0.m
        inputs.velocity = 0.rps
        inputs.voltage = 0.volts
        inputs.current = 0.amps
        inputs.statorCurrent = 0.amps
        inputs.absoluteEncoderPositionNoOffset = 0.deg
        inputs.controlModeValue = 0
    }

    override fun updateInputs() {
        motor.update(Timer.getTimestamp())
        if (logConfig.current) inputs.current = motor.appliedCurrent
        if (logConfig.statorCurrent)
            inputs.statorCurrent = motor.appliedCurrent * 2.0

        if (logConfig.voltage) inputs.voltage = motor.appliedVoltage
        if (logConfig.velocity) inputs.velocity = motor.velocity
        if (logConfig.position) {
            inputs.position = motor.position.rot
            inputs.distance = inputs.position.toDistance(diameter, gearRatio)
        }
        if (gains.hasPIDChanged()) {
            controller.p = gains[GainsEnum.KP]
            controller.i = gains[GainsEnum.KI]
            controller.d = gains[GainsEnum.KD]
            simGains.kV = gains[GainsEnum.KV]
            profiledPIDController.p = gains[GainsEnum.KP]
            profiledPIDController.i = gains[GainsEnum.KI]
            profiledPIDController.d = gains[GainsEnum.KD]
            motor.setController(controller)
        }
        if (gains.hasMotionMagicChanged()) {
            profiledPIDController.constraints =
                TrapezoidProfile.Constraints(
                    gains[GainsEnum.CRUISE_VELOCITY],
                    gains[GainsEnum.ACCELERATION],
                )
        }
    }
}
