package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.log
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.AngularVelocity

object Flywheel : Mechanism() {
    val mainMotor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )

    val auxMotor1 =
        UniversalTalonFX(
            port = FIRST_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )
    val auxMotor2 =
        UniversalTalonFX(
            port = SECOND_AUX_MOTOR_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            logConfig = MOTOR_LOG_CONFIG,
        )

    var setpoint = 0.deg_ps
    val atSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint, TOLERANCE)
    }

    private val velocityVoltage = VelocityVoltage(0.0)

    init {
        addPeriodic(::periodic)
        auxMotor1.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
        )
        auxMotor2.setControl(
            Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
        )
    }

    fun setVelocity(velocity: AngularVelocity): Command =
        this {
                setpoint = velocity
                mainMotor.setControl(velocityVoltage.withVelocity(velocity))
                waitUntil(atSetpoint)
            }
            .named("Subsystems/Flywheel/setVelocity")

    fun periodic() {
        mainMotor.periodic()
        auxMotor1.periodic()
        auxMotor2.periodic()

        mapOf(
                "atSetpoint" to atSetpoint,
                "setpoint" to setpoint,
                "setpointError" to setpoint - mainMotor.inputs.velocity
            )
            .log("Subsystem/Flywheel")
    }
}
