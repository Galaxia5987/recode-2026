package frc.robot.subsystems.turret

import com.ctre.phoenix6.CANBus.systemcore
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.rot
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.system.Timer
import org.wpilib.units.measure.Angle
import kotlin.math.sin

object Turret : Mechanism() {
    private val absoluteEncoder = CANcoder(ENCODER_ID, systemcore(3))
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS,
        )

    var setpoint: Angle = 0.deg
    val positionVoltage = PositionVoltage(0.0)
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    fun setAngle(angle: Angle) = this {
        setpoint = angle
        motor.setControl(positionVoltage.withPosition(setpoint))
        waitUntil(atSetpoint)
    }

    fun setAngle(angleSupplier: () -> Angle) = this {
        while(true){
            setpoint = angleSupplier.invoke()
            motor.setControl(positionVoltage.withPosition(setpoint))
            yield()
        }
    }

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
        addPeriodic(::periodic)
    }

    fun periodic(){
        motor.periodic()
    }

}
