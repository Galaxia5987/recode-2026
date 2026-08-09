package frc.robot.subsystems.turret

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.Gains
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism
import org.wpilib.units.measure.Angle

object Turret : Mechanism() {

    private val MAX_ANGLE: Angle = 270.0.deg
    private val MIN_ANGLE: Angle = (-10.0).deg
    private val absoluteEncoder = CANcoder(5, systemcore(3))
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = TalonFXConfiguration(), // what is this
            gearRatio = 0.0,
            simGains = Gains(), // TODO understand
        )

    var targetAngle: Angle = 0.deg

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
    }

    fun setAngle(angle: Angle) {
        var realAngle = angle
        if (realAngle > MAX_ANGLE) {
            realAngle = MAX_ANGLE
        }
        if (realAngle < MIN_ANGLE) {
            realAngle = MIN_ANGLE
        }
        targetAngle = angle
        motor.setControl(PositionVoltage(angle))
    }

    fun getAngle(): Angle {
        return motor.inputs.position
    }

    fun turretLimits() {
        if (getAngle() < MAX_ANGLE && getAngle() > MIN_ANGLE) {}
    }
}
