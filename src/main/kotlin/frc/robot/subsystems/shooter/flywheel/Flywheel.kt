package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.log
import frc.robot.lib.extensions.mps
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Velocity

object Flywheel : Mechanism() {
    val motors: Array<UniversalTalonFX> = arrayOf(
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG
        ),
        UniversalTalonFX(
            port = FIRST_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG
        ),
        UniversalTalonFX(
            port = SECOND_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            logConfig = MOTOR_LOG_CONFIG

        )
    )
    val mainMotor = motors[0]

    var setpoint = 0.deg_ps
    val atSetpoint = Trigger {
        motors.all { motor ->
            motor.inputs.velocity.isNear(setpoint, TOLERANCE)
        }
    }

    private val velocityVoltage = VelocityVoltage(0.0)

    init {
        addPeriodic(::periodic)
        motors[1].setControl(Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned))
        motors[2].setControl(Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned))
    }

    fun setVelocity(velocity: AngularVelocity) : Command =
        this {
            setpoint = velocity
            mainMotor.setControl(velocityVoltage.withVelocity(velocity))

            waitUntil(atSetpoint)
        }.named("Subsystems/Flywheel")

    fun periodic() {
        motors.forEach { motor -> motor.periodic() }

        mapOf(
            "atSetpoint" to atSetpoint,
            "setpoint" to setpoint
        )
            .forEach { (key, value) -> value.log("Subsystem/Flywheel", key)}
    }
}