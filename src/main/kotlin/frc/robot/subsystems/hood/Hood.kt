package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import frc.robot.lib.commands.addPeriodic
import frc.robot.lib.commands.invoke
import frc.robot.lib.convertTo360
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.log
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.wpilib.command3.Command
import org.wpilib.command3.Mechanism
import org.wpilib.command3.Trigger
import org.wpilib.math.util.Units
import org.wpilib.units.measure.Angle

class Hood : Mechanism() {
    val hoodMotor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
        )

    init {
        addPeriodic(::periodic)
    }

    private var positionRequest = PositionVoltage(0.deg)
    private var setPoint = 0.deg
    val atSetpoint = Trigger {
        hoodMotor.inputs.position.isNear(setPoint, TOLERANCE)
    }
    var positionDegrees = 0.deg

    fun goToPosition(): Command =
        this {
                hoodMotor.setControl(positionRequest.withPosition(setPoint))
            }
            .named("subsystems/Hood/goToPosition")

    fun setSetpoint(angle: Angle) : Command =
        this{
            setPoint = angle
        }.named("subsystems/Hood/setSetpoint")


    fun periodic() {
        hoodMotor.periodic()
        mapOf(
                "atSetpoint" to atSetpoint,
                "setpoint" to setPoint,
                "setpointError" to setPoint - hoodMotor.inputs.position,
            )
            .log("Subsystems/hood")
        Logger.recordOutput("Subsystems/Hood/degreePosition", Units.radiansToDegrees(hoodMotor.inputs.position.baseUnitMagnitude)
        )
    }
}
