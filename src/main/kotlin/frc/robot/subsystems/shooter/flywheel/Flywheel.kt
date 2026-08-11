package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.VelocityVoltage
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.mps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Trigger
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Velocity

object Flywheel {
    val motors: Array<UniversalTalonFX> = arrayOf(
        UniversalTalonFX(
            port = PORT
        ),
        UniversalTalonFX(
            port = PORT
        ),
        UniversalTalonFX(
            port = PORT
        )
    )

    val setpoint = 0.deg_ps
    val atSetpoint = Trigger {
        motors.all { motor ->
            motor.inputs.velocity.isNear(setpoint, TOLERANCE)
        }
    }

    private val velocityVoltage = VelocityVoltage(0.0)

    fun setVelocity(velocity: AngularVelocity) {
        motors.forEach { motor ->
            motor.setControl(velocityVoltage.withVelocity(velocity))
        }
    }
}